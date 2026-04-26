import argparse

from mmwave_uart import *  # noqa: F401,F403


def main():
    ap = argparse.ArgumentParser(description="TI mmWave UART starter parser")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6")
    ap.add_argument("--cfg-file", help="Path to .cfg file")
    ap.add_argument("--data-port", help="COM port for DATA UART, e.g. COM5")
    ap.add_argument("--frames", type=int, default=10, help="Number of frames to read")
    ap.add_argument("--save-raw", help="Optional file path to save raw packets")
    ap.add_argument("--self-test", action="store_true", help="Run parser self-test and exit")
    args = ap.parse_args()

    if args.self_test:
        run_self_test()
        return

    if not args.data_port:
        ap.error("--data-port is required unless --self-test is used")

    if args.cfg_port and args.cfg_file:
        send_cfg(args.cfg_port, args.cfg_file)

    raw_out = open(args.save_raw, "wb") if args.save_raw else None
    reader = MmwaveUartParser(args.data_port)

    try:
        for _ in range(args.frames):
            frame = reader.read_frame()
            if frame is None:
                print("[DATA] Timed out or incomplete frame.")
                continue

            if raw_out:
                raw_out.write(frame["raw_packet"])

            print_frame(frame)

    finally:
        reader.close()
        if raw_out:
            raw_out.close()


if __name__ == "__main__":
    main()
