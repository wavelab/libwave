# wave/utils/filesystem.hpp


## Functions

    bool file_exists(const std::string &file_name);

Checks to see if `file_name` exists, returns boolean.

---

    std::vector<std::string> path_split(const std::string path);

Splits `path` with the `/` token, returns separated path elements in the form of `std::vector<std::string>`.

---

    void paths_combine(const std::string path1,
                       const std::string path2,
                       std::string &out);

Combines `path1` with `path2`, the result is written to `out`. This function is modeled after python's `os.path.join` function. Example usage:

    paths_combine("/a/b", "c/d", out) --> out = "/a/b/c/d"
    paths_combine("/a/b", "../", out) --> out = "/a"
    paths_combine("/a/b/c", "../..", out) --> out = "/a"
