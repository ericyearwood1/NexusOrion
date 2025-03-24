# (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

import json
import os
import sys

repos_dirty = False
manifest_dirty = False


# todo automate what we can, output url for docs

# https://serverfault.com/questions/50775/how-do-i-change-my-private-key-passphrase
# https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
# https://docs.unity3d.com/Manual/upm-git.html#authentication
def ensure_ssh_credentials():
    print("Setting up credentials for ghe.oculus-rep.com...")

    # git config --global user.name "USER NAME"
    # git config --global user.email "yourname@fb.com"

    # MAC
    # brew install git-lfs
    # ssh -T git@ghe.oculus-rep.com
    # ssh-keygen -t ed25519 -C "yourname@fb.com"
    ### no passphrase!
    # eval "$(ssh-agent -s)"
    ### or just `ssh-agent` ?
    # ssh-add -K ~/.ssh/id_ed25519
    # pbcopy < ~/.ssh/id_ed25519.pub
    # https://ghe.oculus-rep.com/settings/keys
    # ssh -T git@ghe.oculus-rep.com

    # WIN
    # <open Git Bash terminal>
    # ssh -T git@ghe.oculus-rep.com
    # ssh-keygen -t ed25519 -C "yourname@fb.com"
    ### no passphrase!
    # eval "$(ssh-agent -s)"
    # ssh-add ~/.ssh/id_ed25519
    # https://ghe.oculus-rep.com/settings/keys
    ### Create a new SSH key, copy paste contents of ~/.ssh/id_ed25519.pub
    # ssh -T git@ghe.oculus-rep.com

    # ssh-keygen -p
    # ssh-keygen -p -f ~/.ssh/id_dsa
    pass


def run_git(repo_path, *args):
    import subprocess

    cmd = ["git"]
    if repo_path:
        cmd = cmd + ["-C", repo_path]
    cmd = cmd + list(args)
    print(" ".join(cmd))
    output = subprocess.check_output(cmd)
    return output.decode(sys.stdout.encoding).strip()


def ensure_local_clone(
    project_root,
    name,
    url,
    branch=None,
    clone_relpath=None,
    clone_packages=None,
    **kwargs,
):
    if not clone_packages:
        return
    if clone_relpath is None:
        clone_relpath = "../.."
    if branch is None:
        branch = "master"

    cloned_repo_path = os.path.abspath(
        os.path.join(project_root, "Packages", clone_relpath, name)
    )

    global repos_dirty
    if not os.path.exists(cloned_repo_path):
        repos_dirty = True
        print(
            f"{name} not found.  Attempting to clone from {url} to {cloned_repo_path}"
        )
        print(f"Make sure you have git access to the {name} repo:")
        print(url)
        run_git(".", "clone", "--branch", branch, url, cloned_repo_path)
        # todo do we need to initialize git-lfs ?
        return

    run_git(cloned_repo_path, "pull")

    # todo is this the optimal sequence to get us update?
    current_branch = run_git(cloned_repo_path, "rev-parse", "--abbrev-ref", "HEAD")
    if branch != current_branch:
        repos_dirty = True
        print(f"Resetting from {current_branch} to {branch}")
        run_git(cloned_repo_path, "stash")
        run_git(cloned_repo_path, "checkout", branch)


packagename_to_folder_special_cases = {"com.osig.tools.ui": "com.osig.ui", "com.osig.tools.ui.examples": "com.osig.ui.examples"}

def update_package_in_manifest(manifest, package_name, package_reference):
    global manifest_dirty

    if package_name in packagename_to_folder_special_cases:
        package_reference = package_reference.replace(
            package_name, packagename_to_folder_special_cases[package_name]
        )

    if package_reference != manifest["dependencies"].get(package_name):
        print("Writing manifest: " + package_reference)
        manifest["dependencies"][package_name] = package_reference
        manifest_dirty = True


def remove_package_in_manifest(manifest, package_name):
    global manifest_dirty
    if manifest["dependencies"].get(package_name):
        package_reference = manifest["dependencies"].pop(package_name)
        print("Removing from manifest: " + package_reference)
        manifest_dirty = True


def update_manifest(
    project_root,
    name,
    url,
    branch=None,
    clone_relpath=None,
    clone_package_subfolder=None,
    clone_packages=None,
    ssh_packages=None,
    unenforced_packages=None,
    **kwargs,
):
    if clone_relpath is None:
        clone_relpath = "../.."
    if clone_package_subfolder is None:
        clone_package_subfolder = "Packages"
    if clone_packages is None:
        clone_packages = []
    if ssh_packages is None:
        ssh_packages = []
    if unenforced_packages is None:
        unenforced_packages = []

    manifest_path = os.path.join(project_root, "Packages/manifest.json")
    with open(manifest_path, "r+") as f:
        manifest = json.load(f)

        for package in clone_packages:
            ref = f"file:{clone_relpath}/{name}/{clone_package_subfolder}/{package}"
            update_package_in_manifest(manifest, package, ref)

        for package in ssh_packages:
            ref = f"{url}?path=/Packages/{package}"
            if branch is not None:
                ref += f"#{branch}"
            update_package_in_manifest(manifest, package, ref)

        for package in unenforced_packages:
            remove_package_in_manifest(manifest, package)

        global manifest_dirty
        if manifest_dirty:
            print("Writing to manifest.json...")
            f.seek(0)  # reset file position to the beginning.
            json.dump(manifest, f, indent=2)
            f.truncate()  # remove remaining part
