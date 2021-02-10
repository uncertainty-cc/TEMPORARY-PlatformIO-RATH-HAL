from os.path import isdir, join

from SCons.Script import DefaultEnvironment, Import

env = DefaultEnvironment()

board = env.BoardConfig()

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-rath-hal")
assert FRAMEWORK_DIR and isdir(FRAMEWORK_DIR)

env.Append(
    ASFLAGS = ["-x", "assembler-with-cpp"],
    CCFLAGS=[
        "-Os",
        "-Wall", 
        "-march=%s" % board.get("build.march"),
        "-mabi=%s" % board.get("build.mabi"),
        "-mcmodel=%s" % board.get("build.mcmodel"),
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common"
    ],
    CFLAGS = [
        "-std=gnu11"
    ],
    CXXFLAGS = [
        "-std=gnu++17"
    ],
    CPPDEFINES = [
        "PLATFORM_RATH_HAL",
        ("HXTAL_VALUE", "%sU" % board.get("build.hxtal_value"))
    ],
    LINKFLAGS=[
        "-march=%s" % board.get("build.march"),
        "-mabi=%s" % board.get("build.mabi"),
        "-mcmodel=%s" % board.get("build.mcmodel"),
        "-nostartfiles",
        "-Xlinker",
        "--gc-sections",
        "--specs=nano.specs"
    ],
    LIBS=["c"]
)

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])


env.Append(
    CPPPATH=[
        join(FRAMEWORK_DIR, "GD32VF1xx_HAL_Driver"),
        join(FRAMEWORK_DIR, "GD32VF1xx_HAL_Driver", "include"),
        join(FRAMEWORK_DIR, "RMSIS", "drivers"),
        join(FRAMEWORK_DIR, "RMSIS"),
    ],
    LIBS=[
        "c"
    ]
)

if not env.BoardConfig().get("build.ldscript", ""):
    env.Replace(
        LDSCRIPT_PATH=join(FRAMEWORK_DIR, "RMSIS", "linkers", board.get("build.RATH-HAL.ldscript"))
    )

#
# Target: Build Core Library
#

libs = [
    env.BuildLibrary(
        join("$BUILD_DIR", "HAL_Driver"),
        join(FRAMEWORK_DIR, "GD32VF1xx_HAL_Driver")),
    env.BuildLibrary(
        join("$BUILD_DIR", "RMSIS"),
        join(FRAMEWORK_DIR, "RMSIS")),
]

env.Prepend(LIBS=libs)
