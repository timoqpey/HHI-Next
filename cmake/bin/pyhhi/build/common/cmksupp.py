from __future__ import print_function

import logging
import os
import sys
import re
import subprocess

import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
import pyhhi.build.common.bldtools as bldtools
from pyhhi.build.common.system import SystemInfo


class CmakeFinder(object):

    def __init__(self, sys_info=None):
        self._logger = logging.getLogger(__name__)
        if sys_info is None:
            self._sys_info = SystemInfo()
        else:
            self._sys_info = sys_info
        self._cmake_prog = None
        self._cmake_version = None
        self._cmake_search_path = self._sys_info.get_path()
        if self._sys_info.is_windows():
            # cmake 3.6.1 is 64 bit but earlier cmake versions are 32 bit only.
            cmake_dir_list = []
            if self._sys_info.get_os_arch() == 'x86_64':
                cmake_dir_list.append(os.path.join(self._sys_info.get_program_dir('x86_64'), 'CMake', 'bin'))
            cmake_dir_list.append(os.path.join(self._sys_info.get_program_dir('x86'), 'CMake', 'bin'))
            for cmake_dir in cmake_dir_list:
                if os.path.exists(cmake_dir):
                    if cmake_dir not in self._cmake_search_path:
                        self._cmake_search_path.append(cmake_dir)
        elif self._sys_info.is_macosx():
            # The default installation path is /Applications/CMake.app/Contents/bin on MacOSX.
            cmake_dir = os.path.join('/Applications', 'CMake.app', 'Contents', 'bin')
            if os.path.exists(cmake_dir):
                if cmake_dir not in self._cmake_search_path:
                    self._cmake_search_path.append(cmake_dir)
        elif self._sys_info.is_linux():
            pass
        else:
            assert False

    def find_cmake(self):
        """Returns the absolute path of a cmake executable."""
        if self._cmake_prog is None:
            self._cmake_prog = util.find_tool_on_path('cmake', must_succeed=True, search_path=self._cmake_search_path)
            self._cmake_version = self._get_cmake_version(self._cmake_prog)
        return self._cmake_prog

    def _get_cmake_version(self, cmake_cmd):
        retv = subprocess.check_output([cmake_cmd, '--version'], universal_newlines=True)
        return self._parse_cmake_version_retv(retv)

    def _parse_cmake_version_retv(self, retv):
        version_response = retv.rstrip()
        lines = version_response.splitlines()
        # Possible version information:
        #   cmake version 3.7.2
        #   cmake version 3.8.0-rc2
        #   cmake3 version 3.6.3           # redhat 7.x cmake3
        re_version_match = re.match(r'cmake\d*\s+version\s+([0-9]+\.[0-9]+\.[0-9]+)', lines[0], re.IGNORECASE)
        if not re_version_match:
            raise Exception("cmake has returned an unsupported version string. Please contact technical support.")
        self._logger.debug("cmake version: %s", re_version_match.group(1))
        return ver.version_tuple_from_str(re_version_match.group(1))


class CmakeLauncherParams(object):

    def __init__(self):
        self.dry_run = False
        self.cmk_build = False
        self.clean_first = False
        self.cmk_build_jobs = 1
        self.cmk_build_target = None
        self.cmk_generator_alias = None
        self.cmk_warning_flags = []
        self.cmk_cache_entries = []
        # gcc, gcc-4.9, clang, msvc, msvc-19.0
        self.toolset_str = None
        self.target_arch = None
        # tuple/list of build configurations (debug, release)
        self.build_configs = tuple(['debug'])
        self.link_variants = tuple(['static'])


class CmakeCompilerInfo(object):

    def __init__(self):
        # If a toolchain file is given, all other attributes are not used and a cross compiler configuration
        # is assumed.
        self.cmake_toolchain_file = None
        # gcc, clang, msvc
        self.compiler_family = None
        self.version_major_minor = None
        self.target_arch = None
        self.cmake_cxx_compiler = None
        self.cmake_c_compiler = None

    def is_cross_compiler(self):
        return self.cmake_toolchain_file is not None

    def __str__(self):
        if self.cmake_toolchain_file:
            s = "toolchain file: %s\n" % self.cmake_toolchain_file
        else:
            s = "compiler family: %s\n" % self.compiler_family
            s += "compiler version (major.minor): %s\n" % ver.version_tuple_to_str(self.version_major_minor)
            if self.target_arch:
                s += "target arch: %s\n" % self.target_arch
            if self.cmake_cxx_compiler:
                s += "cmake cxx compiler: %s\n" % self.cmake_cxx_compiler
                s += "cmake c compiler:   %s\n" % self.cmake_c_compiler
        # if self.build_dirs:
        #    s += "build dir: %s\n" % self.build_dirs
        return s


class CmakeBuildTreeInfo(object):

    def __init__(self, build_root, compiler_info, generator_alias):
        self._logger = logging.getLogger(__name__)
        self._sys_info = SystemInfo()
        self._build_root = build_root
        self._generator_alias = generator_alias
        self._default_build_configs = ['debug', 'release', 'relwithdebinfo', 'minsizerel']
        # key=bld_variant.lnk_variant value=build_dir
        self._build_dir_dict = self._create_build_dir_dict(compiler_info, generator_alias, self._default_build_configs)

    def get_build_dir(self, bld_config, lnk_variant):
        return self._build_dir_dict[bld_config + '.' + lnk_variant]

    def is_multi_configuration_generator(self):
        return self._generator_alias.startswith('vs') or (self._generator_alias == 'xcode')

    def _create_build_dir_dict(self, compiler_info, generator_alias, build_configs):
        build_dir_dict = {}
        if compiler_info.is_cross_compiler():
            assert generator_alias == 'umake'
            build_root = self._create_cross_compile_build_dir(self._build_root, compiler_info.cmake_toolchain_file, generator_alias)
            print("cross compile build root: " + build_root)
            if generator_alias == 'umake':
                for cfg in build_configs:
                    build_dir_dict[cfg + '.' + 'static'] = os.path.join(build_root, cfg)
                    build_dir_dict[cfg + '.' + 'shared'] = os.path.join(build_root, cfg + '-shared')
        else:
            target_arch = compiler_info.target_arch
            toolset_dir = compiler_info.compiler_family + '-' + ver.version_tuple_to_str(compiler_info.version_major_minor)
            if self.is_multi_configuration_generator():
                for cfg in build_configs:
                    build_dir_dict[cfg + '.' + 'static'] = os.path.join(self._build_root, generator_alias, toolset_dir, target_arch)
                    build_dir_dict[cfg + '.' + 'shared'] = os.path.join(self._build_root, generator_alias, toolset_dir, target_arch + '-shared')
            else:
                for cfg in build_configs:
                    build_dir_dict[cfg + '.' + 'static'] = os.path.join(self._build_root, generator_alias, toolset_dir, target_arch, cfg)
                    build_dir_dict[cfg + '.' + 'shared'] = os.path.join(self._build_root, generator_alias, toolset_dir, target_arch, cfg + '-shared')
        return build_dir_dict

    def _create_cross_compile_build_dir(self, build_root, toolchain_file, generator_alias):
        assert generator_alias == 'umake'
        basenm = os.path.basename(toolchain_file)
        re_match = re.match(r'(.+)\.cmake$', basenm)
        if re_match:
            basenm = re_match.group(1)
        if self._sys_info.is_linux():
            # Try to strip a distro related suffix.
            distro_suffix = self._sys_info.get_os_distro_short()
            re_match = re.match('(.+)[-]' + distro_suffix + '([-_.0-9]+)?$', basenm)
            if re_match:
                basenm = re_match.group(1)
        build_dir = os.path.join(build_root, basenm)
        return build_dir


class CmakeLauncher(object):

    def __init__(self, verbosity=1):
        self._logger = logging.getLogger(__name__)
        self._sys_info = SystemInfo()
        self._verbosity_level = verbosity
        self._cmake_finder = CmakeFinder(self._sys_info)
        self._top_dir = None
        self._build_root = None
        self._build_tree_create_if_not_exists = True
        self._build_tree_info = None
        self._deploy_dir = None
        self._script_name = os.path.basename(sys.argv[0])
        # cache entries the user cannot override via -Dxxx
        self._cmk_reserved_cache_vars = ['BUILD_SHARED_LIBS',
                                         'CMAKE_BUILD_TYPE',
                                         'CMAKE_CONFIGURATION_TYPES',
                                         'CMAKE_CXX_COMPILER',
                                         'CMAKE_C_COMPILER',
                                         'CMAKE_TOOLCHAIN_FILE']
        self._dict_to_cmake_generator = {'umake': 'Unix Makefiles',
                                         'mgwmake': 'MinGW Makefiles',
                                         'ninja': 'Ninja',
                                         'xcode': 'Xcode',
                                         'vs14': 'Visual Studio 14 2015',
                                         'vs12': 'Visual Studio 12 2013',
                                         'vs11': 'Visual Studio 11 2012',
                                         'vs10': 'Visual Studio 10 2010'}

        # list of default configuration types for multiconfiguration generators.
        # self._default_config_types = ['debug', 'release']
        self._default_config_types = ['debug', 'release', 'relwithdebinfo']

        self._dict_to_cmake_config = {'debug': 'Debug', 'release': 'Release', 'relwithdebinfo': 'RelWithDebInfo', 'minsizerel': 'MinSizeRel'}
        self._dict_to_vs_platform_name = {'x86_64': 'x64', 'x86': 'Win32'}
        self._dict_to_vs_platform_toolset = {'msvc-19.0': 'v140',
                                             'msvc-18.0': 'v120',
                                             'msvc-17.0': 'v110',
                                             'msvc-16.0': 'v100'}
        self._dict_generator_alias_to_msvc_toolsets = {'vs14': ['msvc-19.0', 'msvc-18.0', 'msvc-17.0', 'msvc-16.0'],
                                                       'vs12': ['msvc-18.0', 'msvc-17.0', 'msvc-16.0'],
                                                       'vs11': ['msvc-17.0', 'msvc-16.0'],
                                                       'vs10': ['msvc-16.0']}

    def get_optimal_number_cmake_jobs(self):
        """Returns the optimal number of cmake jobs."""
        cmake_jobs = self._sys_info.get_number_processors()
        if 'CMAKE_MAX_JOBS' in os.environ:
            cmake_max_jobs = int(os.environ['CMAKE_MAX_JOBS'], 10)
            if cmake_jobs > cmake_max_jobs:
                cmake_jobs = cmake_max_jobs
        assert cmake_jobs >= 1
        return cmake_jobs

    def launch(self, params, cmake_argv):
        self._top_dir = os.getcwd()
        self._build_root = os.path.join(self._top_dir, 'build')
        self._deploy_dir = os.path.join(self._top_dir, 'deploy')
        if not os.path.exists(self._build_root):
            os.makedirs(self._build_root)
        self._check_cmake_params(params)
        compiler_info = self._create_compiler_info(params.toolset_str, params.target_arch)
        #print(compiler_info)
        #return
        self._build_tree_info = CmakeBuildTreeInfo(self._build_root, compiler_info, params.cmk_generator_alias)
        #print(self._build_tree_info.get_build_dir('release', 'static'))
        #print(self._build_tree_info.get_build_dir('release', 'shared'))
        #print(self._build_tree_info.get_build_dir('debug', 'static'))
        #print(self._build_tree_info.get_build_dir('debug', 'shared'))
        # return
        if params.cmk_build:
            # cmake build
            if self._build_tree_create_if_not_exists and (not self._is_build_target_clean(params.cmk_build_target)):
                # cleaning a non-existing build tree is really a no-op and does not require a build tree at all.
                cmake_argv_config = []
                if params.cmk_warning_flags:
                    cmake_argv_config.extend(params.cmk_warning_flags)
                if params.cmk_cache_entries:
                    cmake_argv_config.extend(params.cmk_cache_entries)
                for lnk in params.link_variants:
                    self._create_default_build_tree(compiler_info, params.cmk_generator_alias, params.build_configs, lnk, cmake_argv_config)
            for lnk in params.link_variants:
                self.launch_build(params, lnk, cmake_argv)
        else:
            # cmake build tree create/update
            for lnk in params.link_variants:
                cmake_argv_config = []
                if params.cmk_warning_flags:
                    cmake_argv_config.extend(params.cmk_warning_flags)
                if params.cmk_cache_entries:
                    cmake_argv_config.extend(params.cmk_cache_entries)
                if cmake_argv:
                    cmake_argv_config.extend(cmake_argv)
                # print("warning flags: ", params.cmk_warning_flags)
                # print("additional flags: ", cmake_argv_config)
                self.launch_config(compiler_info, params.cmk_generator_alias, params.build_configs, lnk, cmake_argv_config)

    def launch_config(self, compiler_info, generator_alias, build_configs, lnk_variant, cmake_argv_optional):
        cur_dir = os.getcwd()
        cmake_argv = []
        if self._is_multi_configuration_generator():
            tmp_build_configs = [build_configs[0]]
        else:
            tmp_build_configs = build_configs
        for cfg in tmp_build_configs:
            b_dir = self._build_tree_info.get_build_dir(cfg, lnk_variant)
            if not os.path.exists(b_dir):
                os.makedirs(b_dir)
            if generator_alias.startswith('vs'):
                cmake_argv = ['-G', self._dict_to_cmake_generator[generator_alias],
                              '-A', self._dict_to_vs_platform_name[compiler_info.target_arch],
                              '-T', self._dict_to_vs_platform_toolset['msvc-' + ver.version_tuple_to_str(compiler_info.version_major_minor)]]
            elif generator_alias == 'xcode':
                cmake_argv = ['-G', self._dict_to_cmake_generator[generator_alias]]
            elif generator_alias in ['umake', 'mgwmake', 'ninja']:
                cmake_argv = ['-G', self._dict_to_cmake_generator[generator_alias],
                              '-DCMAKE_BUILD_TYPE=' + self._dict_to_cmake_config[cfg]]
                if compiler_info.is_cross_compiler():
                    cmake_argv.append('-DCMAKE_TOOLCHAIN_FILE=' + compiler_info.cmake_toolchain_file)
                else:
                    if compiler_info.cmake_cxx_compiler:
                        cmake_argv.append('-DCMAKE_CXX_COMPILER=' + compiler_info.cmake_cxx_compiler)
                    if compiler_info.cmake_c_compiler:
                        cmake_argv.append('-DCMAKE_C_COMPILER=' + compiler_info.cmake_c_compiler)
            if cmake_argv_optional:
                # Add any additional arguments to the cmake command line.
                cmake_argv.extend(cmake_argv_optional)
            if lnk_variant == 'shared':
                cmake_argv.append('-DBUILD_SHARED_LIBS=1')
            if self._is_multi_configuration_generator():
                cmake_config_types = [self._dict_to_cmake_config[x] for x in self._default_config_types]
                for b_cfg in build_configs:
                    if b_cfg not in self._default_config_types:
                        cmake_config_types.append(self._dict_to_cmake_config[b_cfg])
                cmake_argv.append('-DCMAKE_CONFIGURATION_TYPES=' + ';'.join(cmake_config_types))
            # cmake_argv.append(self._top_dir)
            # print("launch_config(): cmake_args", cmake_argv)
            # print("build dir:", b_dir)
            # print("top dir:", self._top_dir)
            os.chdir(b_dir)
            cmake_argv.append(os.path.relpath(self._top_dir))
            retv = self.launch_cmake(cmake_argv)
            os.chdir(cur_dir)
            if retv != 0:
                sys.exit(1)

    def launch_build(self, params, lnk_variant, cmake_argv_optional):
        if self._is_multi_configuration_generator():
            # multiple configurations / build directory
            b_dir = self._build_tree_info.get_build_dir(params.build_configs[0], lnk_variant)
            if self._is_build_target_clean(params.cmk_build_target) and (not os.path.exists(b_dir)):
                return
            for cfg in params.build_configs:
                cmake_argv = ['--build', b_dir, '--config', self._dict_to_cmake_config[cfg]]
                self._add_common_cmake_build_options(cmake_argv, params)
                self._add_cmake_build_jobs_option(cmake_argv, params.cmk_generator_alias, params.cmk_build_jobs)
                if cmake_argv_optional:
                    self._add_cmake_build_tool_options(cmake_argv, cmake_argv_optional)
                retv = self.launch_cmake(cmake_argv)
                if retv != 0:
                    sys.exit(1)
        else:
            # one build directory / configuration
            for cfg in params.build_configs:
                b_dir = self._build_tree_info.get_build_dir(cfg, lnk_variant)
                if self._is_build_target_clean(params.cmk_build_target) and (not os.path.exists(b_dir)):
                    continue
                cmake_argv = ['--build', b_dir]
                self._add_common_cmake_build_options(cmake_argv, params)
                self._add_cmake_build_jobs_option(cmake_argv, params.cmk_generator_alias, params.cmk_build_jobs)
                if cmake_argv_optional:
                    self._add_cmake_build_tool_options(cmake_argv, cmake_argv_optional)
                retv = self.launch_cmake(cmake_argv)
                if retv != 0:
                    sys.exit(1)

    def launch_cmake(self, cmake_argv):
        argv = [self._cmake_finder.find_cmake()]
        argv.extend(cmake_argv)

        if self._verbosity_level > 0:
            # assemble the cmake command line for logging purposes
            joiner = ' '
            cmd_line = joiner.join(argv)
            print("Launching: " + cmd_line)
        retv = subprocess.call(argv)
        if retv < 0:
            self._logger.debug("child was terminated by signal: %d", -retv)
        else:
            self._logger.debug("child returned: %d", retv)
        return retv

    def _is_build_target_clean(self, target):
        return (target is not None) and (target == 'clean')

    def _check_cmake_params(self, params):
        if params.cmk_generator_alias is None:
            params.cmk_generator_alias = self._get_default_cmake_generator()
        if params.toolset_str is None:
            if self._sys_info.get_platform() == 'linux':
                params.toolset_str = 'gcc'
            elif self._sys_info.get_platform() == 'macosx':
                params.toolset_str = 'clang'
            elif self._sys_info.get_platform() == 'windows':
                if params.cmk_generator_alias == 'mgwmake':
                    params.toolset_str = 'gcc'
                else:
                    params.toolset_str = self._dict_generator_alias_to_msvc_toolsets[params.cmk_generator_alias][0]
            else:
                assert False
        elif params.toolset_str == 'msvc':
            # toolset=msvc means to select the latest msvc version the selected generator supports.
            assert self._sys_info.get_platform() == 'windows'
            params.toolset_str = self._dict_generator_alias_to_msvc_toolsets[params.cmk_generator_alias][0]
        elif params.toolset_str.startswith('msvc-'):
            if params.toolset_str not in self._dict_generator_alias_to_msvc_toolsets[params.cmk_generator_alias]:
                raise Exception("The selected generator does not support " + params.toolset_str + ", check toolset and generator arguments.")
        if params.target_arch is None:
            params.target_arch = self._sys_info.get_os_arch()
        if params.cmk_cache_entries:
            self._check_cmake_user_cache_entries(params.cmk_cache_entries)

    def _check_cmake_user_cache_entries(self, user_cache_entries):
        # -D<var>:<type>=<value> or -D<var>=<value> provided by the user cannot override any reserved cache entries like BUILD_SHARED_LIBS, ...
        re_cache_entry = re.compile(r'-D(\w+)[=:]')
        for cache_opt in user_cache_entries:
            re_match = re_cache_entry.match(cache_opt)
            if re_match:
                cache_var = re_match.group(1)
                if cache_var in self._cmk_reserved_cache_vars:
                    raise Exception("CMake cache entry " + cache_var + " is reserved by "
                                    + self._script_name + " and may not be overridden via -D<expr>, please contact technical support.")
            else:
                # Unexpected -D expression, please investigate.
                assert False

    def _get_default_cmake_generator(self):
        if self._sys_info.get_platform() == 'linux':
            generator_alias = 'umake'
        elif self._sys_info.get_platform() == 'macosx':
            generator_alias = 'xcode'
        elif self._sys_info.get_platform() == 'windows':
            msvc_registry = bldtools.MsvcRegistry(self._sys_info.get_os_arch())
            # e.g. 14.0, 12.0 etc.
            bb_vs_latest_version = msvc_registry.get_latest_version()
            generator_alias = 'vs' + str(bb_vs_latest_version[0])
        else:
            assert False
        return generator_alias

    def _is_multi_configuration_generator(self):
        return self._build_tree_info.is_multi_configuration_generator()

    def _create_compiler_info(self, toolset_str, target_arch):
        compiler_info = CmakeCompilerInfo()
        if toolset_str and (toolset_str.endswith('.cmake')):
            # toolchain file specified -> no need to add any furter compiler details
            compiler_info.cmake_toolchain_file = toolset_str
            return compiler_info
        # native compiler selected or assumed, figure out details to create the build tree folder.
        compiler_info.target_arch = target_arch
        re_msvc_version = re.compile(r'msvc-(\d+\.\d+)$')
        re_match = re_msvc_version.match(toolset_str)
        if re_match:
            compiler_info.compiler_family = 'msvc'
            compiler_info.version_major_minor = ver.version_tuple_from_str(re_match.group(1))
            return compiler_info
        else:
            assert not toolset_str.startswith('msvc')
            bb_toolset_info = bldtools.Toolset(self._sys_info, toolset_str)
        compiler_info.compiler_family = bb_toolset_info.get_toolset()
        compiler_info.version_major_minor = bb_toolset_info.get_version()[:2]
        # re_toolset_versioned = re.compile('([a-z]+)-(\d+\.\d+)$')
        if self._sys_info.get_platform() == 'linux':
            if toolset_str != 'gcc':
                compiler_info.cmake_cxx_compiler = bb_toolset_info.get_compiler_command()
                cxx_basename = os.path.basename(compiler_info.cmake_cxx_compiler)
                print("cxx_basename: ", cxx_basename)
                if compiler_info.compiler_family == 'gcc':
                    gcc_basename = cxx_basename.replace('g++', 'gcc')
                    compiler_info.cmake_c_compiler = os.path.join(os.path.dirname(compiler_info.cmake_cxx_compiler), gcc_basename)
                elif compiler_info.compiler_family == 'clang':
                    clang_basename = cxx_basename.replace('++', '')
                    compiler_info.cmake_c_compiler = os.path.join(os.path.dirname(compiler_info.cmake_cxx_compiler), clang_basename)
        elif self._sys_info.get_platform() == 'macosx':
            assert compiler_info.compiler_family == 'clang'
        elif self._sys_info.get_platform() == 'windows':
            assert (compiler_info.compiler_family == 'msvc') or (compiler_info.compiler_family == 'gcc')
            if compiler_info.compiler_family == 'gcc':
                # MinGW as native compiler: 64 bit and 32 bit default targets are possible.
                compiler_info.target_arch = bb_toolset_info.get_platform_info(0).get_target_arch(0)
        return compiler_info

    def _create_default_build_tree(self, compiler_info, generator_alias, build_configs, lnk_variant, cmake_argv_optional):
        if self._is_multi_configuration_generator():
            build_dir = self._build_tree_info.get_build_dir(build_configs[0], lnk_variant)
            if not os.path.exists(build_dir):
                self.launch_config(compiler_info, generator_alias, build_configs, lnk_variant, cmake_argv_optional)
        else:
            for cfg in build_configs:
                build_dir = self._build_tree_info.get_build_dir(cfg, lnk_variant)
                if not os.path.exists(build_dir):
                    self.launch_config(compiler_info, generator_alias, [cfg], lnk_variant, cmake_argv_optional)

    def _add_common_cmake_build_options(self, cmake_argv, params):
        if params.cmk_build_target:
            cmake_argv.extend(['--target', params.cmk_build_target])
        if params.clean_first:
            cmake_argv.append('--clean-first')

    def _add_cmake_build_jobs_option(self, cmake_argv, generator_alias, build_jobs):
        if build_jobs < 2:
            return
        if generator_alias in ['umake', 'ninja']:
            self._add_cmake_build_tool_options(cmake_argv, ['-j' + str(build_jobs)])
        elif generator_alias.startswith('vs'):
            self._add_cmake_build_tool_options(cmake_argv, ['/maxcpucount:' + str(build_jobs)])
        elif generator_alias == 'xcode':
            self._add_cmake_build_tool_options(cmake_argv, ['-parallelizeTargets', '-jobs', str(build_jobs)])

    def _add_cmake_build_tool_options(self, cmake_argv, build_tool_options):
        if not build_tool_options:
            # no options given -> return
            return
        assert '--' not in build_tool_options
        if '--' not in cmake_argv:
            cmake_argv.append('--')
        cmake_argv.extend(build_tool_options)
