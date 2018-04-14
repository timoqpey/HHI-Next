from __future__ import print_function

import logging
import re
import os


class CMakeRstUtilParams(object):

    def __init__(self):
        self.dry_run = False
        self.rst_module_filenm = None
        self.update_action = 'add'
        self.extension_module_names = []
        self.extension_section_title = "Extension Modules"
        self.output_rst_filenm = None


class RstModuleSection(object):

    def __init__(self):
        self.section_title = None
        self.section_title_marker_line = None
        self.section_header = []
        self.section_content = []


class CMakeManualRstUtil(object):

    def __init__(self, dry_run=False):
        self._logger = logging.getLogger(__name__)
        self._dry_run = dry_run
        self._section_title_cmake_modules_orig = "All Modules"
        self._section_title_cmake_modules = "Standard Modules"
        self._section_title_marker = '='
        self._is_cmake_source_tree = False
        # Location of <module>.rst
        self._cmake_help_module_dir = None
        # Automatic removal of cmake module wrapper files.
        self._autoremove = True
        self._re_section_content = re.compile(r'\s+/module/([a-zA-Z0-9_-]+)\s*$')

    def is_cmake_source_tree(self):
        return self._is_cmake_source_tree

    def add_extension_modules(self, rst_module_filenm, extension_module_names, section_title="Extension Modules", output_rst_filenm=None):
        if not os.path.exists(rst_module_filenm):
            raise Exception("CMake RST module file " + rst_module_filenm + " does not exist.")
        if output_rst_filenm is None:
            output_rst_filenm = rst_module_filenm
        # Empty sections are not permitted, use one of the remove_xxx() methods to get rid of a section.
        assert extension_module_names
        self._detect_cmake_source_tree(rst_module_filenm)
        self._check_section_title(section_title)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)
        if len(section_list) > 0:
            # Get cmake module names
            cmake_module_names_set = self._get_cmake_module_names(section_list[-1])
            # Extension module names must not replace existing cmake module names
            extension_module_names_set = set(extension_module_names)
            if not cmake_module_names_set.isdisjoint(extension_module_names_set):
                conflicting_names = cmake_module_names_set.intersection(extension_module_names_set)
                msg = "CMake builtin module names cannot be part of an extension section.\n"
                msg += "Please check the extension module names: " + " ".join(conflicting_names)
                raise Exception(msg)
        else:
            # Looks like a corrupted cmake-modules.7.rst
            assert False
        # Get a list of RST extension module wrapper files belonging to section "section_title".
        existing_extension_module_names = self._get_extension_module_names(section_list, section_title)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))
        modified = self._update_section(section_list, section_title, extension_module_names)
        if modified:
            # print("new content lines:")
            # self._dump_section_list(section_list[0:-1])
            # Change title of the CMake module section from "All Modules" to "Standard Modules".
            section = section_list[-1]
            if section.section_title == self._section_title_cmake_modules_orig:
                section.section_title = self._section_title_cmake_modules
                section.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(section.section_title))
            section_list = self._sort_section_list(section_list)
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, section_list)
            # Remove RST extension module wrapper files not belonging to this section anymore.
            outdated_extension_module_names = []
            for mod_nm in existing_extension_module_names:
                if mod_nm not in extension_module_names:
                    outdated_extension_module_names.append(mod_nm)
            if outdated_extension_module_names:
                self._logger.debug('outdated extensions: %s', ' '.join(outdated_extension_module_names))
                # Remove outdated RST extension module wrapper files.
                self._remove_rst_extension_module_files(outdated_extension_module_names)

    def remove_extension_module_section(self, rst_module_filenm, section_title="Extension Modules", output_rst_filenm=None):
        if not os.path.exists(rst_module_filenm):
            raise Exception("CMake RST module file " + rst_module_filenm + " does not exist.")
        if output_rst_filenm is None:
            output_rst_filenm = rst_module_filenm
        self._detect_cmake_source_tree(rst_module_filenm)
        self._check_section_title(section_title)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)
        assert len(section_list) > 0
        if len(section_list) <= 1:
            return
        # Get a list of RST extension module wrapper files belonging to section "section_title".
        existing_extension_module_names = self._get_extension_module_names(section_list, section_title)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))
        modified = False
        new_section_list = []
        for sect in section_list[0:-1]:
            if section_title == sect.section_title:
                modified = True
            else:
                new_section_list.append(sect)
        # Any extensions left?
        if new_section_list:
            new_section_list = self._sort_section_list(new_section_list)
        new_section_list.append(section_list[-1])
        if len(new_section_list) == 1:
            sect = new_section_list[-1]
            if sect.section_title != self._section_title_cmake_modules_orig:
                modified = True
                # Recover the original section title
                sect.section_title = self._section_title_cmake_modules_orig
                sect.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(sect.section_title))
        if modified:
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, new_section_list)
            # Remove RST extension module wrapper files belonging to the section just removed.
            self._remove_rst_extension_module_files(existing_extension_module_names)

    def remove_all_extension_modules(self, rst_module_filenm, output_rst_filenm=None):
        if not os.path.exists(rst_module_filenm):
            raise Exception("CMake RST module file " + rst_module_filenm + " does not exist.")
        if output_rst_filenm is None:
            output_rst_filenm = rst_module_filenm
        self._detect_cmake_source_tree(rst_module_filenm)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)
        existing_extension_module_names = self._get_extension_module_names(section_list)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))
        modified = False
        if len(section_list) > 1:
            modified = True
        # Get and keep the last section, it's supposed to be the original CMake module section.
        section = section_list[-1]
        if section.section_title != self._section_title_cmake_modules_orig:
            modified = True
            # recover the original section title
            section.section_title = self._section_title_cmake_modules_orig
            section.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(section.section_title))
        if modified:
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, [section])
            # Remove all RST extension module wrapper files
            self._remove_rst_extension_module_files(existing_extension_module_names)

    def _detect_cmake_source_tree(self, rst_module_filenm):
        rst_manual_dir = os.path.dirname(rst_module_filenm)
        rst_module_dir = os.path.join(rst_manual_dir, '..', 'module')
        if (os.path.basename(rst_manual_dir) == 'manual') and (os.path.exists(rst_module_dir) and os.path.isdir(rst_module_dir)):
            self._is_cmake_source_tree = True
            self._cmake_help_module_dir = self._get_rst_module_dir(rst_module_filenm)
            assert os.path.exists(self._cmake_help_module_dir) and os.path.isdir(self._cmake_help_module_dir)
        else:
            self._is_cmake_source_tree = False
            self._cmake_help_module_dir = None

    def _get_rst_module_dir(self, rst_module_filenm):
        rst_manual_dir = os.path.dirname(rst_module_filenm)
        rst_module_dir = os.path.join(rst_manual_dir, '..', 'module')
        return os.path.normpath(rst_module_dir)

    def _check_section_title(self, section_title):
        if section_title in [self._section_title_cmake_modules_orig, self._section_title_cmake_modules]:
            raise Exception("section " + section_title + " is protected and cannot be modified, please contact technical support.")

    def _check_extension_module_names(self, extension_module_names):
        if not self.is_cmake_source_tree():
            return
        for mod_nm in extension_module_names:
            rst_module_file = os.path.join(self._cmake_help_module_dir, mod_nm + '.rst')
            if not os.path.exists(rst_module_file):
                raise Exception("file " + rst_module_file + " does not exist.")

    def _update_section(self, section_list, section_title, extension_module_names):
        section = None
        modified = False
        assert extension_module_names
        extension_module_names.sort()
        for sec in section_list:
            if sec.section_title == section_title:
                section = sec
                break
        if not section:
            # add a new section
            modified = True
            section = RstModuleSection()
            section.section_title = section_title
            section.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(section.section_title))
            section.section_header.append('')
            section.section_header.append('.. toctree::')
            section.section_header.append('   :maxdepth: 1')
            section.section_header.append('')
            section_list.insert(0, section)
        else:
            # Section already present, update only if section content is different.
            existing_extension_module_names = []
            # re_section_content = re.compile(r'\s+/module/([a-zA-Z0-9_-]+)\s*$')
            for line in section.section_content:
                re_match = self._re_section_content.match(line)
                if re_match:
                    existing_extension_module_names.append(re_match.group(1))
            existing_extension_module_names.sort()
            if len(existing_extension_module_names) != len(extension_module_names):
                modified = True
            else:
                for module_nm in existing_extension_module_names:
                    if module_nm not in extension_module_names:
                        modified = True
                        break
            if modified:
                # replace the section
                section.section_content = []
        if modified:
            # replace the content of an existing section
            for module_nm in extension_module_names:
                section.section_content.append('   /module/' + module_nm)
        return modified

    def _sort_section_list(self, section_list):
        # Sort section list by title and keep the last section at the end.
        section_title_dict = {}
        section_list_sorted = []
        for sect in section_list[0:-1]:
            section_title_dict[sect.section_title] = sect
        title_list = list(section_title_dict.keys())
        title_list.sort()
        for title in title_list:
            section_list_sorted.append(section_title_dict[title])
        # and preserve the position of the last element.
        section_list_sorted.append(section_list[-1])
        return section_list_sorted

    def _parse_rst_module_file(self, rst_module_filenm):
        section_list = []
        module_file_header = []
        re_section_title = re.compile(r'^[a-zA-Z][a-zA-Z0-9 -]+')
        # re_section_content = re.compile(r'\s+/module/')
        re_rst_directive = re.compile(r'^\s*\.\.\s+[a-zA-Z0-9-]+')
        re_empty_line = re.compile(r'^\s*$')
        with open(rst_module_filenm) as f:
            for line in f:
                line = line.rstrip()
                if not section_list:
                    # Processing lines in front of the first section header
                    if re_rst_directive.match(line):
                        module_file_header.append(line)
                    elif re_empty_line.match(line):
                        module_file_header.append(line)
                    elif line.startswith('cmake-modules') or line.startswith('***'):
                        module_file_header.append(line)
                    elif re_section_title.match(line):
                        # print("found 1st section: " + line)
                        section = RstModuleSection()
                        section.section_title = line
                        section_list.append(section)
                        # Remove the last empty line from the module header to simplify saving sections later on.
                        module_file_header.pop()
                else:
                    if line.startswith(self._section_title_marker):
                        section.section_title_marker_line = line
                    elif re_section_title.match(line):
                        # Start of a new section
                        # print("found new section: " + line)
                        section = RstModuleSection()
                        section.section_title = line
                        section_list.append(section)
                    elif self._re_section_content.match(line):
                        section.section_content.append(line)
                    elif not section.section_content:
                        section.section_header.append(line)
        # self._dump_module_file_header(module_file_header)
        # self._dump_section_list(section_list)
        return module_file_header, section_list

    def _get_extension_module_names(self, section_list, section_title=None):
        extension_module_names = []
        for sect in section_list:
            if sect.section_title in [self._section_title_cmake_modules_orig, self._section_title_cmake_modules]:
                continue
            if (section_title is None) or (section_title == sect.section_title):
                for line in sect.section_content:
                    re_match = self._re_section_content.match(line)
                    if re_match:
                        extension_module_names.append(re_match.group(1))
        return extension_module_names

    def _get_cmake_module_names(self, sect):
        cmake_module_names = set()
        for line in sect.section_content:
            re_match = self._re_section_content.match(line)
            if re_match:
                cmake_module_names.add(re_match.group(1))
        return cmake_module_names

    def _save_rst_module_file(self, output_rst_filenm, module_file_header, section_list):
        if self._dry_run:
            self._logger.debug("leaving without saving anything to %s", output_rst_filenm)
            return
        print("Saving updated content to " + output_rst_filenm)
        with open(output_rst_filenm, "w") as f:
            for line in module_file_header:
                f.write(line + '\n')
            for sect in section_list:
                f.write('\n')
                f.write(sect.section_title + '\n')
                f.write(sect.section_title_marker_line + '\n')
                for line in sect.section_header:
                    f.write(line + '\n')
                for line in sect.section_content:
                    f.write(line + '\n')

    def _remove_rst_extension_module_files(self, extension_module_names):
        if not self._autoremove:
            return
        if not self.is_cmake_source_tree():
            return
        for mod_nm in extension_module_names:
            rst_module_file = os.path.join(self._cmake_help_module_dir, mod_nm + '.rst')
            if os.path.exists(rst_module_file):
                self._logger.debug("removing %s", rst_module_file)
                if not self._dry_run:
                    os.remove(rst_module_file)

    def _dump_module_file_header(self, module_file_header):
        for line in module_file_header:
            print("file header: " + line)

    def _dump_section_list(self, section_list):
        print("number of sections: %d" %(len(section_list)))
        for section in section_list:
            print("section title:        " + section.section_title)
            print("section title marker: " + section.section_title_marker_line)
            for line in section.section_header:
                print("section header: " + line)
            for line in section.section_content:
                print("section content: " + line)