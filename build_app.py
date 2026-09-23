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
    app_elf = os.path.join(ROOT, "app.elf")
    run(["arm-none-eabi-gcc", "-nostartfiles", "-T", APP_LD,
         "-Wl,--gc-sections", "-Wl,--no-warn-rwx-segments",
         "-o", app_elf, libapp, "-lgcc"])

    # 3) objcopy -> app.bin
    run(["arm-none-eabi-objcopy", "-O", "binary", app_elf, args.out])
    sz = os.path.getsize(args.out)
    print(f"[OK] app.bin 产出: {args.out} ({sz} bytes, 须 < {APP_FLASH_BUDGET})")
    sync_to_test_artifact(args.out, app_elf)
    if sz > APP_FLASH_BUDGET:
        print(f"[ERR] App 镜像超过 APP_FLASH {APP_FLASH_BUDGET // 1024}K 预算", file=sys.stderr)
        sys.exit(1)

def sync_to_test_artifact(out: str, app_elf: str) -> None:
    """★把构建产物同步到【M 场实际加载的路径】✓（`mcu_simulater/src/artifact.rs:143`）。

    为何 ✓：M 场加载的是 `/tmp/flyctrl_real.bin`（或 `flyctrl/app_real.bin`）✗，
    而不是 `app.bin` ✗ —— 曾因此出现"重建了固件、M 场却仍在测旧货"✗✓（§5.29 教训 ✓）。
    ⇒ 统一构建脚本负责保证【输出文件名/路径一致】✓，避免再踩 ✗。
    """
    dsts = ["/tmp/flyctrl_real.bin",
            os.path.join(os.path.dirname(app_elf), "app_real.bin")]
    for dst in dsts:
        try:
            shutil.copyfile(out, dst)
            print(f"[SYNC] {dst} ({os.path.getsize(dst)} bytes) ✓")
        except OSError as e:  # 只读/权限等 ⇒ 明确告警，不静默 ✗
            print(f"[WARN] 同步到 {dst} 失败：{e}", file=sys.stderr)


if __name__ == "__main__":
    main()
