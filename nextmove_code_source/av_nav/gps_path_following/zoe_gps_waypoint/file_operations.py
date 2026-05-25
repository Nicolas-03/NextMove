from typing import Optional
from typing import TextIO
from pathlib import Path

SUCCESS : bool = True
FAILURE : bool = False

def extract_file_name_from_path(path_str: str) -> str:
    path = Path(path_str)
    return path.name

def close_file(fd: TextIO) -> bool:
    try:
        fd.close()
    except Exception as e:
        print("Close failed:", e)
        return FAILURE
    return SUCCESS

def close_files(fds: list[TextIO]):
    for fd in fds:
        try:
            fd.close()
        except Exception as e:
            print("Close failed:", e)

def open_file(f : str) -> Optional[TextIO]:
    try:
        path = Path(f).expanduser()
        if not path.is_absolute():
            path = path.resolve()
        fd = open(path, 'r')
        return fd
    except FileNotFoundError:
        print(f"File or path doesn't exist: {path}")
    return None

def open_files(filenames: list[str]) -> list[TextIO]:
    """
        Open each file in the given list of absolute file paths_ and store their file objects in `fds`.
        Args:
            filenames (list[str]): list of paths_ to the files to open.
            fds (list[IO]): list to be filled with the opened file objects.
    """
    fds : list[TextIO] = []
    for filename in filenames:
        fd = open_file(filename)
        if fd is not None:
            print(f"{filename} successfully opened.")
            fds.append(fd)
    return fds
