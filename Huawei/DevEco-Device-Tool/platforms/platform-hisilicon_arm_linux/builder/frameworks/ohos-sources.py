from os.path import join
import subprocess
from SCons.Script import Import
from SCons import Errors
from platformio.util import get_systype

Import("env")

builder_env = env  # noqa: F821  # pylint: disable=undefined-variable

platform = builder_env.PioPlatform()
board = builder_env.BoardConfig()
framework_settings = board.get("frameworks").get('ohos-sources').get("build")


def message(target, source, env):  # pylint: disable=unused-argument
    if 'windows' in get_systype():
        return "warning: can't build on windows"

    cwd = env.subst("$PROJECT_DIR")
    fullcmd = ["./build.sh", "--product-name", framework_settings.get("product_name")]
    return 'cd ' + cwd + ' && ' + ' '.join(fullcmd)


def builder(target, source, env):  # pylint: disable=unused-argument,unreachable
    if 'windows' in get_systype():
        raise Errors.BuildError(errstr='Can not build on windows')

    fullcmd = ["./build.sh", "--product-name", framework_settings.get("product_name")]

    cp = subprocess.run(fullcmd,
                        cwd=env.subst("$PROJECT_DIR")
                        )
    return cp.returncode

builder_env.Append(BUILDERS=dict(RepoBuilder=builder_env.Builder(
    action=builder_env.Action(builder, message))))

builder_env.Replace(
    FRAMEWORK_BUILD_TARGET=framework_settings.get("target"),
)

builder_env.Replace(
    BUILD_OUT_DIR=join("$PROJECT_DIR", "out", framework_settings.get("target")))

builder_env.Replace(
    BUILD_ELF_TARGET=join("$BUILD_OUT_DIR", builder_env.subst("${PROGNAME}")),
    PROG_PATH='$BUILD_ELF_TARGET',
)

program = builder_env.RepoBuilder(['$BUILD_ELF_TARGET'], None)
builder_env.Replace(PIOMAINPROG=program)