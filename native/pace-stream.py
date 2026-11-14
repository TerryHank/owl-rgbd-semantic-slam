"""Replay a recorded OWL1 stream at its original capture rate. No ROS imports."""
import sys
import time


def main():
    with open(sys.argv[1], "rb") as source:
        header = source.readline()
        magic, width, height, *_ = header.split()
        if magic != b"OWL1":
            raise ValueError("Expected OWL1 recording")
        size = int(width) * int(height) * 7
        out = sys.stdout.buffer
        out.write(header)
        out.flush()
        first = None
        start = time.monotonic()
        for stamp in iter(source.readline, b""):
            timestamp = float(stamp)
            if first is None:
                first = timestamp
                start = time.monotonic()
            data = source.read(size)
            if len(data) != size:
                raise ValueError("Truncated frame")
            delay = start + timestamp - first - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            out.write(stamp)
            out.write(data)
            out.flush()


if __name__ == "__main__":
    main()
