from __future__ import print_function

import logging
import os
import sys
import re
import platform
import traceback


def exec_main_default_try(main_fnc, sys_exit_err=1, finally_action=None):
    """Execute main_fnc inside a try block and dump the callstack in case of execptions."""
    exit_error = False
    try:
        main_fnc()
    except KeyboardInterrupt:
        exit_error = True
        #print("Keyboard interrupt signaled")
    except Exception:
        exit_error = True
        exc_type, exc_value, exc_traceback = sys.exc_info()
        lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
        for line in lines[:-1]:
            print(line.rstrip())
        print('-----')
        print(lines[-1])
    finally:
        if finally_action:
            finally_action()
    if exit_error:
        # some exception thrown, exit with error code to inform the shell something went wrong.
        sys.exit(sys_exit_err)
    return 0


def app_args_add_log_level(parser):
    parser.add_argument("--log-level", action="store", dest="log_level", choices=['warning', 'debug'], default="warning",
                        help="configure the log level [default: warning]")


def app_configure_logging(log_level):
    # assuming loglevel is bound to the string value obtained from the
    # command line argument. Convert to upper case to allow the user to
    # specify --log-level=DEBUG or --log-level=debug
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % log_level)

    #FORMAT = "py-trace %(module)s.%(funcName)s: %(lineno)d: %(message)s"
    FORMAT = "%(module)s.%(funcName)s: %(lineno)d: %(message)s"
    logging.basicConfig(format=FORMAT, level=numeric_level)


def normalize_path(fpath):
    # remove leading and trailing spaces
    fpath = fpath.lstrip().rstrip()
    if platform.system().lower() == 'windows':
        # ensure drive letters are in uppercase which os.path.normpath() does not seem to enforce.
        re_drive_letter = re.compile(r'([a-z]):', re.IGNORECASE)
        re_match = re_drive_letter.match(fpath)
        if re_match:
            fpath = re_match.group(1).upper() + fpath[1:]
    return os.path.normpath(fpath)


def find_tool_on_path(tool, must_succeed=False, search_path=None):
    """Find a tool on the search path and return the full path."""
    if os.path.isabs(tool):
        if platform.system().lower() == 'windows':
            tool_basename = os.path.basename(tool)
            if not re.search(r'\.\S+$', tool_basename):
                tool = os.path.join(os.path.dirname(tool), tool_basename + '.exe')
        if os.path.exists(tool):                
            return tool
    else:
        if platform.system().lower() == 'windows':
            # special fix for windows to qualify tool with .exe if tool does not have an extension.
            if not re.search(r'\.\S+$', tool):
                tool += '.exe'
        if search_path is None:
            search_path = os.path.expandvars('$PATH')
            search_path = search_path.split(os.path.pathsep)
        for d in search_path:
            if d == '.':
                continue
            prog = os.path.join(d, tool)            
            if os.path.exists(prog) and (not os.path.isdir(prog)):
                # Always return an absolute path in case the current working directory will changed later on.
                prog = os.path.abspath(prog)
                return prog
    if must_succeed:
        if os.path.isabs(tool):
            raise Exception("The command '" + tool + "' does not exist.")
        else:
            raise Exception("The command " + tool + " cannot be found on PATH.")
    return None


def get_tool_dir(tool):
    if platform.system().lower() == 'windows':
        # some folks use slashes on windows or combination of slashes and backslashes
        re_pathsep = re.compile(r'[\\/:]+')
    else:
        # assume a linux path
        re_pathsep = re.compile(r'[/]+')
    if re_pathsep.search(tool):
        tool_path = normalize_path(os.path.abspath(tool))
    else:
        tool_path = find_tool_on_path(tool)
    # Resolve any symbolic links to get the real location; e.g.
    # bjam.py is a symbolic link and points to $HOME/bin/bjam.py. The real location of bjam.py is writeable but
    # the location of the symbolic link may not.
    tool_path = os.path.realpath(tool_path)
    tool_dir = os.path.dirname(tool_path)
    return tool_dir


def get_top_dir(script=None):
    # git layout with submodules:
    #   script_dir = <top>/submodules/BoostBuild/BoostBuild/bin
    #   script_dir = <top>/submodules/SomeOtherRepo/SomeOtherRepo/bin
    #                <top>/submodules/SomeOtherRepo/submodules -> exists!!!!
    #   script_dir = <top>/BoostBuildTest/bin
    #
    # svn layout with submodules:
    #   script_dir = <top>/submodules/BoostBuild/BoostBuild/bin
    #   script_dir = <top>/submodules/SomeOtherRepo/SomeOtherRepo/bin
    #   script_dir = <top>/BoostBuildTest/bin
    #
    # svn layout without submodules:
    #   script_dir = <top>/BoostBuild/bin
    #   script_dir = <top>/BoostBuildTest/bin
    if script is None:
        script_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
    else:
        script_dir = os.path.dirname(os.path.abspath(script))
    submodules_dir = os.path.normpath(os.path.join(script_dir, '..', '..', '..', '..', 'submodules'))
    submodules_dir2 = os.path.normpath(os.path.join(script_dir, '..', '..', 'submodules'))
    boost_build_bin_path = os.path.join('BoostBuild', 'BoostBuild', 'bin')
    if os.path.exists(os.path.join(submodules_dir, boost_build_bin_path)):
        top_dir = os.path.normpath(os.path.join(submodules_dir, '..'))
        boost_build_bin_dir = os.path.join(submodules_dir, boost_build_bin_path)
    elif os.path.exists(os.path.join(submodules_dir2, boost_build_bin_path)):
        top_dir = os.path.normpath(os.path.join(submodules_dir2, '..'))
        boost_build_bin_dir = os.path.join(submodules_dir2, boost_build_bin_path)
    else:
        top_dir = os.path.normpath(os.path.join(script_dir, '..', '..'))
        boost_build_bin_dir = os.path.join(top_dir, 'BoostBuild', 'bin')
    # now validate top_dir in case something went wrong above
    if not os.path.exists(boost_build_bin_dir):
        raise Exception(os.path.basename(sys.argv[0]) + " failed to find its parent workspace, please contact technical support.")
    return top_dir


def get_boost_build_dir(top_dir):
    boost_build_dir = os.path.join(top_dir, 'submodules', 'BoostBuild', 'BoostBuild')
    if not os.path.exists(boost_build_dir):
        # assume the previous SVN layout without a submodules folder holding the externals
        boost_build_dir = os.path.join(top_dir, 'BoostBuild')
    if not os.path.exists(boost_build_dir):
        raise Exception("The directory '" + top_dir + "' does not seem to be a workspace directory with a BoostBuild folder, please contact technical support.")
    return boost_build_dir


def get_boost_build_script_dir(top_dir):
    boost_build_dir = get_boost_build_dir(top_dir)
    script_dir = os.path.join(boost_build_dir, 'bin')
    return script_dir


def find_repo_path_from_src_path(src_path):
    src_path = os.path.abspath(src_path.rstrip(os.path.sep))
    is_windows = platform.system().lower() == 'windows'
    # split the path
    if is_windows:
        drive_path_comps = os.path.splitdrive(src_path)
        src_path = drive_path_comps[1]

    dir_list = src_path.lstrip(os.path.sep).split(os.path.sep)
    joiner = os.path.sep

    while dir_list:
        # walk the tree until 'src' is found. The parent of src is supposed to be the name of the repository.
        dir = dir_list.pop()
        if len(dir_list) < 2:
            break

        if dir.lower() == 'src':
            if is_windows:
                if (len(dir_list) > 3) and (dir_list[-3] == 'submodules'):
                    jamroot_dir = os.path.join(drive_path_comps[0], os.path.sep, joiner.join(dir_list[:-3]))
                else:
                    jamroot_dir = os.path.join(drive_path_comps[0], os.path.sep, joiner.join(dir_list[:-1]))
                repo_path = os.path.join(drive_path_comps[0], os.path.sep, joiner.join(dir_list))
            else:
                # repo_path: /home/rauthenberg/projects/BoostBuild/BoostBuild
                # or
                # repo_path: /home/rauthenberg/projects/BoostBuildTest/submodules/BoostBuild/BoostBuild
                if (len(dir_list) > 3) and (dir_list[-3] == 'submodules'):
                    jamroot_dir = os.path.join(os.path.sep, joiner.join(dir_list[:-3]))
                else:
                    jamroot_dir = os.path.join(os.path.sep, joiner.join(dir_list[:-1]))
                repo_path = os.path.join(os.path.sep, joiner.join(dir_list))
            if os.path.isfile(os.path.join(jamroot_dir, 'Jamroot')):
                return repo_path
    raise Exception("The repository path cannot be deduced from '" + src_path + "'")


def find_repo_name_from_src_path(src_path):
    repo_path = find_repo_path_from_src_path(src_path)
    if platform.system().lower() == 'windows':
        drive_path_comps = os.path.splitdrive(repo_path)
        repo_path = drive_path_comps[1]
    return repo_path.lstrip(os.path.sep).split(os.path.sep)[-1]


def find_src_file_in_submodules(top_dir, src_file):
    if not os.path.exists(os.path.join(top_dir, 'submodules')):
        return src_file
    if os.path.isabs(src_file) and (not os.path.exists(src_file)):
        top_dir = normalize_path(top_dir)
        src_file = normalize_path(src_file)
        # vf=/home/rauthenberg/projects/BoostBuildTest/BoostBuild/include/BoostBuild/version.h
        # top=/home/rauthenberg/projects/BoostBuildTest
        # ->
        # vf=/home/rauthenberg/projects/BoostBuildTest/submodules/BoostBuild/BoostBuild
        if src_file.startswith(top_dir):
            src_file = src_file[len(top_dir) + 1:]
            src_file_comps = src_file.split(os.path.sep)
            joiner = os.path.sep
            src_file = os.path.join(top_dir, 'submodules', src_file_comps[0], joiner.join(src_file_comps))
    return src_file


