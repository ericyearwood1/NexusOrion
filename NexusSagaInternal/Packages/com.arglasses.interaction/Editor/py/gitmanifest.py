# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

import json, os, sys
from os import listdir
from os.path import basename, dirname, exists, isdir, join

import git

# todo add some meaningful defaults that can be selected from MenuItem in Unity (i.e. sideload ITK.Full)
# empty_git_entry = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gitmanifest.empty.json')
# default_git_entry = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gitmanifest.default.json')


def run(manifest_path):
    project_root = os.path.dirname(os.path.dirname(manifest_path))
    with open(manifest_path, "r") as f:
        manifest = json.load(f)

    repo_list = manifest.get("git")
    if not repo_list:
        print(f"No [git] section in {manifest_path}")
        exit()

    for repo in repo_list:
        git.ensure_ssh_credentials()
        git.ensure_local_clone(project_root, **repo)
        git.update_manifest(project_root, **repo)

    if git.repos_dirty or git.manifest_dirty:
        input("gitmanifest updated.\n\nPress Enter to continue...")


def find_directories(parent_dir):
    return [
        basename(join(parent_dir, d))
        for d in listdir(parent_dir)
        if isdir(join(parent_dir, d))
    ]


def locate_unity_manifest(path=None):
    path = path or os.getcwd()

    while path != dirname(path):
        path = dirname(path)
        candidate_manifest_path = join(path, "Packages", "manifest.json")
        if exists(candidate_manifest_path):
            return candidate_manifest_path


if __name__ == "__main__":
    try:
        print("Running gitmanifest: " + " ".join(sys.argv))

        if len(sys.argv) > 1:
            manifest_path = sys.argv[1]
        else:
            manifest_path = locate_unity_manifest()

        if manifest_path is None:
            print("Could not find a Unity project :(")
            exit()

        run(manifest_path)
        print("git.py success")

    except Exception:
        import traceback

        traceback.print_exc()
        input(
            "git.py exited early.  Your sidecar repos may be in a bad state :(\n\nPress Enter to continue..."
        )
