"""Export the embedded indoor vocabulary for the RK3588/RK3588S NPU."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

def convert(graph, output):
    from rknn.api import RKNN
    converter = RKNN()
    try:
        operations = (
            ("config", lambda: converter.config(target_platform="rk3588",
                      mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]])),
            ("load_onnx", lambda: converter.load_onnx(model=str(graph))),
            ("build", lambda: converter.build(do_quantization=False)),
            ("export", lambda: converter.export_rknn(str(output))),
        )
        for name, run in operations:
            result = run()
            if result != 0:
                raise RuntimeError(f"RKNN {name} failed ({result})")
    finally:
        converter.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path, help="YOLOE .pt checkpoint or previously exported .onnx graph")
    parser.add_argument("output", type=Path)
    parser.add_argument("--rknn-python", type=Path, help="Python interpreter with RKNN Toolkit2 and ONNX 1.16.1")
    args = parser.parse_args()
    if args.checkpoint.suffix == ".onnx":
        convert(args.checkpoint, args.output)
        return
    from ultralytics import YOLOE

    names = json.loads(Path(__file__).with_name("yoloe-indoor-classes.json").read_text(encoding="utf-8"))
    model = YOLOE(str(args.checkpoint))
    model.set_classes(names, model.get_text_pe(names))
    graph = model.export(format="onnx", imgsz=640, opset=17, simplify=True, end2end=False, device="cpu")
    subprocess.run([str(args.rknn_python or sys.executable), __file__, str(graph), str(args.output)], check=True)


if __name__ == "__main__":
    main()
