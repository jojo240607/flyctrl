#!/usr/bin/env python3
"""flyctrl App 分区构建：编译 flyctrl/app → 链接成可烧录到 APP_FLASH 的 app.bin。

流程（复用 joc-rtos-app-sdk 的 linker/app.ld，入口 rust_app_start 由 SDK 提供）：
  1) cargo build --release（flyctrl/app，thumbv7em-none-eabihf）
     -> flyctrl/app/target/thumbv7em-none-eabihf/release/libflyctrl_app.a
  2) arm-none-eabi-gcc -nostartfiles -T <sdk>/linker/app.ld 链接 -> app.elf
  3) arm-none-eabi-objcopy -O binary app.elf app.bin

用法：
  python3 build_app.py                     # 默认（正式飞控）
  python3 build_app.py --features real-sensors   # 真实传感器驱动
  python3 build_app.py --features hil      # 硬件在环
  python3 build_app.py --features demo     # 极简拉起链路验证
  python3 build_app.py --out /path/app.bin
"""
import argparse, os, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.join(ROOT, "app")
SDK_ROOT = os.path.abspath(os.path.join(ROOT, "..", "joc-rtos-app-sdk"))
APP_LD = os.path.join(SDK_ROOT, "linker", "app.ld")
APP_FLASH_BUDGET = 384 * 1024  # APP_FLASH 384KB

def run(cmd, cwd=None):
    print("+", " ".join(cmd), flush=True)
    subprocess.check_call(cmd, cwd=cwd or ROOT)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--features", default="", help="cargo features（逗号分隔；默认空=正式飞控）")
    ap.add_argument("--out", default=os.path.join(ROOT, "app.bin"), help="输出 app.bin 路径")
    ap.add_argument("--check", action="store_true", help="仅校验工具链")
    args = ap.parse_args()

    for t in ("cargo", "arm-none-eabi-gcc", "arm-none-eabi-objcopy"):
        if shutil.which(t) is None:
            print(f"[ERR] 工具未找到: {t}（请加入 PATH）", file=sys.stderr)
            sys.exit(1)
    if args.check:
        print("[OK] 工具链就绪")
        return
    if not os.path.isfile(APP_LD):
        print(f"[ERR] SDK 链接脚本缺失: {APP_LD}", file=sys.stderr)
        sys.exit(1)

    # 1) cargo -> libflyctrl_app.a
    cmd = ["cargo", "build", "--release"]
    if args.features:
        cmd += ["--features", args.features]
    run(cmd, cwd=APP_DIR)
    libapp = os.path.join(APP_DIR, "target", "thumbv7em-none-eabihf", "release", "libflyctrl_app.a")
    if not os.path.exists(libapp):
        print(f"[ERR] 未找到 {libapp}", file=sys.stderr)
        sys.exit(1)

    # 2) 链接（app.ld 生成头部 + 定位到 APP_FLASH/APP_RAM）
    # ★ELF 按 feature 命名（§5.130）：app.elf 曾被每次构建覆盖（last-build-wins），
    #   而 mcu_simulater 的 elfsym::app_sym 从 flyctrl/app.elf 解析符号；
    #   .app_globals 段基址固定但段内偏移随 feature 漂移（实测 hil vs real 的
    #   SENSOR_SEQ 差 +0xE1C）⇒ 加载 real.bin 时若 app.elf 属其它 feature，
    #   符号探针全部错位（x_env_* 家族 100% 失败的根因之一）。
    app_elf = os.path.join(ROOT, f"app{feature_tag(args.features)}.elf")
    run(["arm-none-eabi-gcc", "-nostartfiles", "-T", APP_LD,
         "-Wl,--gc-sections", "-Wl,--no-warn-rwx-segments",
         "-o", app_elf, libapp, "-lgcc"])

    # 3) objcopy -> app.bin
    run(["arm-none-eabi-objcopy", "-O", "binary", app_elf, args.out])
    sz = os.path.getsize(args.out)
    print(f"[OK] app.bin 产出: {args.out} ({sz} bytes, 须 < {APP_FLASH_BUDGET})")
    sync_to_test_artifact(args.out, app_elf, args.features)
    if sz > APP_FLASH_BUDGET:
        print(f"[ERR] App 镜像超过 APP_FLASH {APP_FLASH_BUDGET // 1024}K 预算", file=sys.stderr)
        sys.exit(1)

def feature_tag(features: str) -> str:
    """feature → 产物命名后缀：'' → ''，real-sensors → '_real'，hil → '_hil'。"""
    feats = [f.strip() for f in (features or "").split(",") if f.strip()]
    if not feats:
        return ""
    if feats == ["real-sensors"]:
        return "_real"
    if feats == ["hil"]:
        return "_hil"
    return "_" + feats[0].replace("-", "_")


def sync_to_test_artifact(out: str, app_elf: str, features: str = "") -> None:
    """★把构建产物同步到【M 场实际加载的路径】✓（`mcu_simulater/src/artifact.rs`）。

    为何 ✓：real-sensors 固件的 M 场加载路径是 `/tmp/flyctrl_real.bin`（或
    `flyctrl/app_real.bin`）✗，而不是 `app.bin` ✗ —— 曾因此出现"重建了固件、
    M 场却仍在测旧货"✗✓（§5.29 教训 ✓）。
    ⇒ 统一构建脚本负责保证【输出文件名/路径一致】✓，避免再踩 ✗。

    ★只同步 real-sensors 的 bin ✓（2026-09-24 §5.129 修）：原实现无条件把
    【每次】构建产物都覆盖到 /tmp/flyctrl_real.bin ⇒ do_firmware 顺序 real→hil
    时，hil 构建最后写入，把 HIL 固件覆盖进 real 路径 ⇒ 三个真传感器测试
    实际加载 HIL 固件（确定性失败）。

    ★bin 与 ELF 成对同步 ✓（2026-09-24 §5.130 修）：符号解析（elfsym）必须用
    【同 feature】的 ELF——.app_globals 段内偏移随 feature 漂移，跨 feature
    组合会读错位。real 同步 ELF 到 /tmp/flyctrl_real.elf + flyctrl/app_real.elf
    （artifact::flyctrl_real_app_elf 解析落点）；hil 同步 ELF 供
    JOC_APP_FLYCTRL_ELF / flyctrl_hil_app_elf 使用。默认构建不 sync
    （flyctrl/app.bin + app{tag}.elf 原地即约定路径）。
    """
    feats = {f.strip() for f in (features or "").split(",") if f.strip()}
    elf_dir = os.path.dirname(app_elf)
    pairs: list[tuple[str, str]] = []
    if "real-sensors" in feats:
        pairs = [
            (out, "/tmp/flyctrl_real.bin"),
            (out, os.path.join(elf_dir, "app_real.bin")),
            (app_elf, "/tmp/flyctrl_real.elf"),
            (app_elf, os.path.join(elf_dir, "app_real.elf")),
        ]
    elif "hil" in feats:
        pairs = [
            (app_elf, "/tmp/flyctrl_hil.elf"),
            (app_elf, os.path.join(elf_dir, "app_hil.elf")),
        ]
    for src, dst in pairs:
        try:
            shutil.copyfile(src, dst)
            print(f"[SYNC] {dst} ({os.path.getsize(dst)} bytes) ✓")
        except OSError as e:  # 只读/权限等 ⇒ 明确告警，不静默 ✗
            print(f"[WARN] 同步到 {dst} 失败：{e}", file=sys.stderr)


if __name__ == "__main__":
    main()
