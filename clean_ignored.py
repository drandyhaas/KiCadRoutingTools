#!/usr/bin/env python3
"""List and optionally delete all git-ignored files in the repository."""

import subprocess
import os
import sys
import shutil


def get_ignored_files():
    """Get list of all files ignored by git."""
    result = subprocess.run(
        ['git', 'ls-files', '--ignored', '--exclude-standard', '-o'],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        print(f"Error running git: {result.stderr}")
        sys.exit(1)

    files = [f.strip() for f in result.stdout.strip().split('\n') if f.strip()]
    return files


def get_ignored_dirs():
    """Get list of ignored directories (not captured by ls-files)."""
    result = subprocess.run(
        ['git', 'ls-files', '--ignored', '--exclude-standard', '-o', '--directory'],
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        return []

    dirs = [d.strip().rstrip('/') for d in result.stdout.strip().split('\n') if d.strip()]
    return dirs


def main():
    import argparse
    parser = argparse.ArgumentParser(description="List and delete git-ignored files")
    parser.add_argument('--delete', action='store_true',
                        help="Actually delete the files (default: just list them)")
    parser.add_argument('--yes', '-y', action='store_true',
                        help="Skip confirmation prompt when deleting")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print("Finding git-ignored files and directories...\n")

    files = get_ignored_files()
    dirs = get_ignored_dirs()

    # Separate files from directories for display
    all_items = []
    for f in files:
        if os.path.isfile(f):
            size = os.path.getsize(f)
            all_items.append(('file', f, size))
        elif os.path.isdir(f):
            all_items.append(('dir', f, 0))

    # Add directories found by --directory flag
    for d in dirs:
        if os.path.isdir(d) and not any(item[1] == d for item in all_items):
            all_items.append(('dir', d, 0))

    if not all_items:
        print("No ignored files found.")
        return

    # Sort and display
    all_items.sort(key=lambda x: x[1])

    total_size = 0
    print("Ignored files and directories:")
    print("-" * 60)
    for item_type, path, size in all_items:
        if item_type == 'file':
            total_size += size
            size_str = f"{size:,} bytes"
            if size > 1024*1024:
                size_str = f"{size/1024/1024:.1f} MB"
            elif size > 1024:
                size_str = f"{size/1024:.1f} KB"
            print(f"  [file] {path} ({size_str})")
        else:
            # Calculate directory size
            dir_size = 0
            for root, subdirs, filenames in os.walk(path):
                for filename in filenames:
                    try:
                        dir_size += os.path.getsize(os.path.join(root, filename))
                    except OSError:
                        pass
            total_size += dir_size
            size_str = f"{dir_size:,} bytes"
            if dir_size > 1024*1024:
                size_str = f"{dir_size/1024/1024:.1f} MB"
            elif dir_size > 1024:
                size_str = f"{dir_size/1024:.1f} KB"
            print(f"  [dir]  {path}/ ({size_str})")

    print("-" * 60)
    total_str = f"{total_size:,} bytes"
    if total_size > 1024*1024:
        total_str = f"{total_size/1024/1024:.1f} MB"
    print(f"Total: {len(all_items)} items, {total_str}")

    if not args.delete:
        print("\nTo delete these files, run: python clean_ignored.py --delete")
        return

    # Confirm deletion
    if not args.yes:
        response = input("\nDelete all these files? [y/N] ")
        if response.lower() != 'y':
            print("Aborted.")
            return

    # Delete files and directories
    print("\nDeleting...")
    deleted = 0
    errors = 0
    for item_type, path, _ in all_items:
        try:
            if item_type == 'file' and os.path.isfile(path):
                os.remove(path)
                print(f"  Deleted: {path}")
                deleted += 1
            elif item_type == 'dir' and os.path.isdir(path):
                shutil.rmtree(path)
                print(f"  Deleted: {path}/")
                deleted += 1
        except OSError as e:
            print(f"  Error deleting {path}: {e}")
            errors += 1

    print(f"\nDeleted {deleted} items" + (f", {errors} errors" if errors else ""))


if __name__ == '__main__':
    main()
