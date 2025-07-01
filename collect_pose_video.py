import argparse
from multiprocessing import Process
import subprocess

def run_demo_collect(demo_idx, fps):
    subprocess.run(['python3', 'human_demo_video.py',
                    '--demo_idx', str(demo_idx),
                    '--fps', str(fps)])

def run_human_data(fps,demo_idx):
    subprocess.run(['python3', 'human_demo_pose.py',
                    '--demo_idx', str(demo_idx),
                    '--fps', str(fps)])


def main():
    parser = argparse.ArgumentParser(description='Run both human video capture and pose logger')
    parser.add_argument('--demo_idx', type=int, default=0, help='Demo index')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    args = parser.parse_args()

    p1 = Process(target=run_demo_collect, args=(args.demo_idx, args.fps))
    p2 = Process(target=run_human_data, args=(args.fps,args.demo_idx))

    p1.start()
    p2.start()

    try:
        p1.join()
        p2.join()
    except KeyboardInterrupt:
        print("Interrupted. Terminating processes...")
        p1.terminate()
        p2.terminate()

if __name__ == '__main__':
    main()
