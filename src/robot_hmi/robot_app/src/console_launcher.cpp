// A native executable can be opened directly from GNOME's file manager.
// Keep the browser/service lifecycle in start.sh so terminal and double-click use the same code.
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
    int fail(const std::string& message)
    {
        std::cerr << "ocs2_control 控制台启动失败：" << message << '\n';
        const pid_t child = fork();
        if (child == 0)
        {
            execlp("notify-send", "notify-send", "ocs2_control 控制台启动失败", message.c_str(),
                   static_cast<char*>(nullptr));
            _exit(127);
        }
        if (child > 0)
        {
            while (waitpid(child, nullptr, 0) == -1 && errno == EINTR) {}
        }
        return 1;
    }
} // namespace

int main()
{
    std::error_code error;
    const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error)
    {
        return fail("无法定位启动文件：" + error.message());
    }
    const auto root = executable.parent_path();
    const auto script = root / "scripts" / "start.sh";
    if (!std::filesystem::is_regular_file(script, error))
    {
        return fail("找不到 " + script.string() + "，请将 ocs2_control控制台 保留在仓库根目录。");
    }
    if (chdir(root.c_str()) != 0)
    {
        return fail("无法进入仓库目录：" + std::string(std::strerror(errno)));
    }
    // Pass arguments directly; paths containing spaces or shell characters are not shell code.
    execl("/bin/bash", "bash", script.c_str(), "console", "--open", static_cast<char*>(nullptr));
    return fail("无法执行 start.sh：" + std::string(std::strerror(errno)));
}
