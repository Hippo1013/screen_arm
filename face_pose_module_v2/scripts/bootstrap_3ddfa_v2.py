from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


DEFAULT_REPO = "https://github.com/cleardusk/3DDFA_V2.git"
DEFAULT_COMMIT = "1b6c67601abffc1e9f248b291708aef0e43b55ae"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Clone the official 3DDFA_V2 repo into face_pose_module_v2.")
    parser.add_argument("--repo", default=DEFAULT_REPO)
    parser.add_argument("--commit", default=DEFAULT_COMMIT)
    parser.add_argument("--target", default="third_party/3DDFA_V2")
    parser.add_argument("--keep-git", action="store_true", help="Keep nested .git directory after checkout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    module_dir = Path(__file__).resolve().parents[1]
    target = Path(args.target)
    if not target.is_absolute():
        target = module_dir / target
    target = target.resolve()

    if target.exists() and any(target.iterdir()):
        print(f"Target already exists and is not empty: {target}")
        return 0

    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.check_call(["git", "clone", "--depth", "1", args.repo, str(target)])

    if args.commit:
        subprocess.check_call(["git", "-C", str(target), "fetch", "--depth", "1", "origin", args.commit])
        subprocess.check_call(["git", "-C", str(target), "checkout", args.commit])

    commit = subprocess.check_output(["git", "-C", str(target), "rev-parse", "HEAD"], text=True).strip()
    (module_dir / "third_party" / "3DDFA_V2_COMMIT.txt").write_text(commit + "\n", encoding="ascii")

    git_dir = target / ".git"
    if git_dir.exists() and not args.keep_git:
        remove_git_dir(git_dir, module_dir)
    print(f"3DDFA_V2 ready at: {target}")
    print(f"Commit: {commit}")
    return 0


def remove_git_dir(git_dir: Path, module_dir: Path) -> None:
    git_dir = git_dir.resolve()
    module_dir = module_dir.resolve()
    if module_dir not in git_dir.parents:
        raise RuntimeError(f"Refusing to remove unexpected path: {git_dir}")
    if sys.platform.startswith("win"):
        subprocess.check_call(["powershell", "-NoProfile", "-Command", f"Remove-Item -LiteralPath '{git_dir}' -Recurse -Force"])
    else:
        subprocess.check_call(["rm", "-rf", str(git_dir)])


if __name__ == "__main__":
    raise SystemExit(main())
