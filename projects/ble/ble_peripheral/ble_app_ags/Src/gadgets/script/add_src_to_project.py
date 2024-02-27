# -*- coding:utf-8 -*-

import sys
import xml.dom.minidom
import os


def add_src_to_project(uvprojx_file, file_path):
    dom = xml.dom.minidom.parse(uvprojx_file)
    root = dom.documentElement

    group_nodes = root.getElementsByTagName('Group')
    for i in range(len(group_nodes)):
        if group_nodes[i].getElementsByTagName('GroupName')[0].firstChild.nodeValue == 'gadgets':
            group_node = group_nodes[i]
            files_node = group_node.getElementsByTagName('Files')[0]

            sub_nodes = files_node.childNodes[7:]

            src_files = os.listdir(file_path)
            for src_file in src_files:
                file_name = os.path.basename(src_file)
                if ".c" in file_name:
                    print("file name:", file_name)
                    file_node = dom.createElement('File')

                    enter_node_1 = dom.createElement(None)
                    enter_text_node_1 = dom.createTextNode('\n              ')
                    enter_node_1.appendChild(enter_text_node_1)

                    file_name_node = dom.createElement('FileName')
                    file_name_text_node = dom.createTextNode(file_name)
                    file_name_node.appendChild(file_name_text_node)

                    enter_text_node_2 = dom.createTextNode('\n              ')

                    file_type_node = dom.createElement('FileType')
                    file_type_text_node = dom.createTextNode('1')
                    file_type_node.appendChild(file_type_text_node)

                    enter_node_3 = dom.createElement(None)
                    enter_text_node_3 = dom.createTextNode('\n              ')
                    enter_node_3.appendChild(enter_text_node_3)

                    file_path_node = dom.createElement('FilePath')
                    file_path_text_node = dom.createTextNode("{}\{}".format(file_path, file_name))
                    file_path_node.appendChild(file_path_text_node)

                    enter_node_4 = dom.createElement(None)
                    enter_text_node_4 = dom.createTextNode('\n            ')
                    enter_node_4.appendChild(enter_text_node_4)

                    file_node.appendChild(file_name_node)
                    file_node.appendChild(file_type_node)
                    file_node.appendChild(file_path_node)

                    enter_text_node_5 = dom.createTextNode('\n            ')

                    files_node.appendChild(file_node)

    with open(uvprojx_file, "w+", encoding='utf-8') as f_w:
        dom.writexml(f_w)


def main():
    print("add files")
    uvprojx_file = sys.argv[1]
    file_path = sys.argv[2]
    
    add_src_to_project(uvprojx_file, file_path)


if __name__ == "__main__":
    main()
