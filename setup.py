import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


PROJECT_ROOT = Path(__file__).resolve().parent


class CMakeExtension(Extension):
    """A setuptools extension whose sources are built by the project CMake file."""

    def __init__(self, name: str, source_dir: Path) -> None:
        super().__init__(name, sources=[])
        self.source_dir = str(source_dir.resolve())


class CMakeBuild(build_ext):
    """Invoke CMake for both normal and editable pip installations."""

    def build_extension(self, extension: CMakeExtension) -> None:
        cmake = self._cmake_executable()

        extension_path = Path(self.get_ext_fullpath(extension.name)).resolve()
        extension_dir = extension_path.parent
        extension_dir.mkdir(parents=True, exist_ok=True)

        configuration = "Debug" if self.debug else "Release"
        build_dir = (
            Path(self.build_temp).resolve()
            / extension.name.replace(".", "_")
        )
        build_dir.mkdir(parents=True, exist_ok=True)

        build_environment = os.environ.copy()
        conda_prefix = build_environment.get("CONDA_PREFIX")
        if conda_prefix:
            self._prepend_path(
                build_environment,
                "PKG_CONFIG_PATH",
                str(Path(conda_prefix) / "lib" / "pkgconfig"),
            )
            self._prepend_path(
                build_environment,
                "LD_LIBRARY_PATH",
                str(Path(conda_prefix) / "lib"),
            )

        cmake_arguments = [
            f"-DCMAKE_BUILD_TYPE={configuration}",
            f"-DFR3_SIM_PYTHON_PACKAGE_DIR={extension_dir}",
            "-DFR3_SIM_BUILD_TESTS=OFF",
            f"-DPython3_EXECUTABLE={sys.executable}",
            "-DPython3_FIND_VIRTUALENV=ONLY",
            "-DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON",
        ]
        if conda_prefix:
            cmake_arguments.append(f"-DCMAKE_PREFIX_PATH={conda_prefix}")

        pybind11_dir = self._pybind11_cmake_dir(build_environment)
        if pybind11_dir:
            cmake_arguments.append(f"-Dpybind11_DIR={pybind11_dir}")

        extra_cmake_arguments = build_environment.get("CMAKE_ARGS")
        if extra_cmake_arguments:
            cmake_arguments.extend(shlex.split(extra_cmake_arguments))

        self.announce(
            f"Configuring {extension.name} with CMake ({configuration})",
            level=3,
        )
        subprocess.run(
            [
                cmake,
                "-S",
                extension.source_dir,
                "-B",
                str(build_dir),
                *cmake_arguments,
            ],
            check=True,
            env=build_environment,
        )

        build_arguments = [
            "--build",
            str(build_dir),
            "--config",
            configuration,
            "--target",
            "_fr3_sim",
        ]
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in build_environment:
            parallel = self.parallel or os.cpu_count() or 2
            build_arguments.extend(["--parallel", str(parallel)])

        self.announce(f"Building {extension.name} with CMake", level=3)
        subprocess.run(
            [cmake, *build_arguments],
            check=True,
            env=build_environment,
        )

        if not extension_path.is_file():
            candidates = sorted(extension_dir.glob("_fr3_sim*.so"))
            found = ", ".join(str(path.name) for path in candidates) or "none"
            raise RuntimeError(
                f"CMake completed but did not produce {extension_path.name}. "
                f"Candidates in {extension_dir}: {found}"
            )

    @staticmethod
    def _prepend_path(environment: dict[str, str], name: str, value: str) -> None:
        current = environment.get(name)
        environment[name] = value if not current else value + os.pathsep + current

    @staticmethod
    def _cmake_executable() -> str:
        override = os.environ.get("CMAKE_EXECUTABLE")
        if override:
            path = Path(override).expanduser().resolve()
            if path.is_file() and os.access(path, os.X_OK):
                return str(path)
            raise RuntimeError(f"CMAKE_EXECUTABLE is not executable: {path}")

        # The conda-forge cmake package may install $CONDA_PREFIX/bin/cmake as
        # a Python wrapper. PEP 517 build isolation hides that wrapper's Python
        # module, while the real CMake executable remains usable here.
        conda_prefix = os.environ.get("CONDA_PREFIX")
        if conda_prefix:
            site_packages = (
                Path(conda_prefix)
                / "lib"
                / f"python{sys.version_info.major}.{sys.version_info.minor}"
                / "site-packages"
            )
            candidates = [site_packages / "cmake" / "data" / "bin" / "cmake"]
            candidates.extend(
                sorted(
                    Path(conda_prefix).glob(
                        "lib/python*/site-packages/cmake/data/bin/cmake"
                    )
                )
            )
            for path in candidates:
                if path.is_file() and os.access(path, os.X_OK):
                    return str(path.resolve())

        cmake = shutil.which("cmake")
        if cmake is not None:
            return cmake
        raise RuntimeError(
            "CMake was not found. Activate the mamba environment described "
            "in README.md before running pip install."
        )

    @staticmethod
    def _pybind11_cmake_dir(environment: dict[str, str]) -> str | None:
        result = subprocess.run(
            [sys.executable, "-m", "pybind11", "--cmakedir"],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        if result.returncode != 0:
            return None
        path = result.stdout.strip()
        return path or None


setup(
    ext_modules=[
        CMakeExtension("fr3_control_sim._fr3_sim", PROJECT_ROOT),
    ],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
