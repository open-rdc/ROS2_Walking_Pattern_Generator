#!/usr/bin/python3

import xml.etree.ElementTree as xml
import yaml

import os

# import pprint

# stringのベクトルをfloat（or int）のlistに変換
# def (str_vec, type="float"):
#   """
#   vec = []
#   el = ""
#   for char in str_vec:
#     el = el + char
#     if char == " ":
#       vec.append(int(el) if type=="int" else float(el))
#       el = ""
#   vec.append(int(el) if type=="int" else float(el))
#   el = ""
#   """
#   #return vec
#   return str_vec

# TODO: BringUpとかに置くべきYAMLファイルを読んで、urdf_pathとrobot_nameを宣言するべき。
robot_name = "robotis_op2"
urdf_name = "robotis_op2.urdf"
print("[INFO] Robot name is " + robot_name)
try:
  models_dir_path = "../../robot_description/models/" + robot_name + "/urdf"
  urdf_path = models_dir_path + "/" + urdf_name
  os.listdir(models_dir_path)
except:
  print("[ERROR] robot_description/models/" + robot_name + "/urdf/" + urdf_name + " is not found.")
  quit()

try:
  os.mkdir("../../robot_description/config/" + robot_name)
  print("[INFO] Create robot_description/config/" + robot_name + " dir.")
except:
  print("[WARNING] Already robot_description/config/" + robot_name + " dir.")

yaml_path = os.path.abspath("../../robot_description/config/" + robot_name + "/")

urdf_source = xml.parse(urdf_path)
urdf_root = urdf_source.getroot()


# setup
urdf_all_joints = urdf_root.findall("joint")
all_joints_name = [joint.attrib["name"] for joint in urdf_all_joints]  # joint名list


# type==fixedなjointのname_listと、それを除外したnonfixedなname_listの作成
fixed_joints_name = []
fixed_joints_parent_link_name = []
fixed_joints_num = []
urdf_nonfixed_joints = []
nonfixed_joints_name = []
for i in range(0, len(urdf_all_joints)):
  if urdf_all_joints[i].attrib["type"] == "fixed":
    fixed_joints_name.append(all_joints_name[i])
    fixed_joints_num.append(i)
    fixed_joints_parent_link_name.append(urdf_all_joints[i].find("parent").attrib["link"])
  else:
    urdf_nonfixed_joints.append(urdf_all_joints[i])
    nonfixed_joints_name.append(all_joints_name[i])


# arms, legs groupを作る
# with fixed_joints
joint_obj_list_lists = [[None]]  # joint_object
joint_names_list_lists = [[None]]  # joint_name
child_link_names_list_lists = [[None]]  # joint child_link_name
joint_nums_list_lists = [[None]]  # joint_number (== count)
joint_link_lengths_list_lists = [[None]]  # length between previous_joint to now_joint
joint_postures_list_lists = [[None]]  # posture between previous_joint to now_joint
# without fixed_joints
nonfixed_joint_obj_list_lists = [[None]]
nonfixed_joint_names_list_lists = [[None]]
nonfixed_child_link_names_list_lists = [[None]]
nonfixed_joint_nums_list_lists = [[None]]
nonfixed_joint_unit_vectors_list_lists = [[None]]  # joint unit_vector
nonfixed_joint_link_lengths_list_lists = [[None]] 
nonfixed_joint_postures_list_lists = [[None]]


parent_link_name_multi_child = []
fixed_joint_cnt = 0
fixed_joint_cnt_in_tree = 0

for i in range(0, len(urdf_all_joints)):
  fixed_type = False
  fixed_joint_cnt_in_tree = 0
  parent_link_name = urdf_all_joints[i].find("parent").attrib["link"]
  if parent_link_name in fixed_joints_parent_link_name:
    fixed_type = True
    fixed_joint_cnt = fixed_joint_cnt + 1

  for j in range(0, len(child_link_names_list_lists)):

    for k in range(0, len(child_link_names_list_lists[j])):

      if parent_link_name != child_link_names_list_lists[j][k]:  # parent_nameが一致しない場合

        if  j == len(child_link_names_list_lists)-1  and  k == len(child_link_names_list_lists[j])-1:  # parent_nameが全探索でも見つからないので、新childを新listをappend
          joint_obj_list_lists.append([urdf_all_joints[i]])
          joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])
          child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])
          joint_nums_list_lists.append([i])
          try:
            joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])
            joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])
          except:
            # print("WARNING")
            joint_link_lengths_list_lists.append(["0.0, 0.0, 0.0"])
            joint_postures_list_lists.append(["0.0, 0.0, 0.0"])

          if fixed_type == False:
            nonfixed_joint_obj_list_lists.append([urdf_all_joints[i]])
            nonfixed_joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])
            nonfixed_child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])
            nonfixed_joint_nums_list_lists.append([i-fixed_joint_cnt])
            nonfixed_joint_unit_vectors_list_lists.append([(urdf_all_joints[i].find("axis").attrib["xyz"])])
            nonfixed_joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])
            nonfixed_joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])

          elif fixed_type == True:
            nonfixed_joint_obj_list_lists.append([])
            nonfixed_joint_names_list_lists.append([])
            nonfixed_child_link_names_list_lists.append([])
            nonfixed_joint_nums_list_lists.append([])
            nonfixed_joint_unit_vectors_list_lists.append([])
            nonfixed_joint_link_lengths_list_lists.append([])
            nonfixed_joint_postures_list_lists.append([])

          break
        
        else:
          continue
      
      elif parent_link_name == child_link_names_list_lists[j][k]:  # parent_nameが既存child_nameと一致した場合

        if k == len(child_link_names_list_lists[j])-1:  # parentがlistの末端なので、後ろにくっつける

          if parent_link_name in parent_link_name_multi_child:
            joint_obj_list_lists.append([urdf_all_joints[i]])
            joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])
            child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])
            joint_nums_list_lists.append([i])
            joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])
            joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])

            if fixed_type == False:
              nonfixed_joint_obj_list_lists.append([urdf_all_joints[i]])
              nonfixed_joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])
              nonfixed_child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])
              nonfixed_joint_nums_list_lists.append([i-fixed_joint_cnt])
              nonfixed_joint_unit_vectors_list_lists.append([(urdf_all_joints[i].find("axis").attrib["xyz"])])
              nonfixed_joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])
              nonfixed_joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])

            elif fixed_type == True:
              nonfixed_joint_obj_list_lists.append([])
              nonfixed_joint_names_list_lists.append([])
              nonfixed_child_link_names_list_lists.append([])
              nonfixed_joint_nums_list_lists.append([])
              nonfixed_joint_unit_vectors_list_lists.append([])
              nonfixed_joint_link_lengths_list_lists.append([])
              nonfixed_joint_postures_list_lists.append([])

            break

          else:
            joint_obj_list_lists[j].append(urdf_all_joints[i])
            joint_names_list_lists[j].append(urdf_all_joints[i].attrib["name"])
            child_link_names_list_lists[j].append(urdf_all_joints[i].find("child").attrib["link"])
            joint_nums_list_lists[j].append(i)
            joint_link_lengths_list_lists[j].append(urdf_all_joints[i].find("origin").attrib["xyz"])
            joint_postures_list_lists[j].append(urdf_all_joints[i].find("origin").attrib["rpy"])

            if fixed_type == False:
              nonfixed_joint_obj_list_lists[j].append(urdf_all_joints[i])
              nonfixed_joint_names_list_lists[j].append(urdf_all_joints[i].attrib["name"])
              nonfixed_child_link_names_list_lists[j].append(urdf_all_joints[i].find("child").attrib["link"])
              nonfixed_joint_nums_list_lists[j].append(i-fixed_joint_cnt)
              nonfixed_joint_unit_vectors_list_lists[j].append(urdf_all_joints[i].find("axis").attrib["xyz"])
              nonfixed_joint_link_lengths_list_lists[j].append(urdf_all_joints[i].find("origin").attrib["xyz"])
              nonfixed_joint_postures_list_lists[j].append(urdf_all_joints[i].find("origin").attrib["rpy"])

            break

        else:  # 単parent複childなので、既存parent未満を新listとしてappend, 次に新childを新listとしてappend
          parent_link_name_multi_child.append(parent_link_name)
          
          joint_obj_list_lists.append(joint_obj_list_lists[j][k+1:])
          del joint_obj_list_lists[-2][k+1:]
          joint_obj_list_lists.append([urdf_all_joints[i]])

          joint_names_list_lists.append(joint_names_list_lists[j][k+1:])
          del joint_names_list_lists[-2][k+1:]
          joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])

          child_link_names_list_lists.append(child_link_names_list_lists[j][k+1:])
          del child_link_names_list_lists[-2][k+1:]
          child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])

          joint_nums_list_lists.append(joint_nums_list_lists[j][k+1:])
          del joint_nums_list_lists[-2][k+1:]
          joint_nums_list_lists.append([i])

          joint_link_lengths_list_lists.append(joint_link_lengths_list_lists[j][k+1:])
          del joint_link_lengths_list_lists[-2][k+1:]
          joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])

          joint_postures_list_lists.append(joint_postures_list_lists[j][k+1:])
          del joint_postures_list_lists[-2][k+1:]
          joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])

          if fixed_type == False:
            
            fixed_joint_cnt_in_tree = len(set(fixed_joints_name) & set(joint_names_list_lists[j]))

            nonfixed_joint_obj_list_lists.append(nonfixed_joint_obj_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_obj_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_obj_list_lists.append([urdf_all_joints[i]])

            nonfixed_joint_names_list_lists.append(nonfixed_joint_names_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_names_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_names_list_lists.append([urdf_all_joints[i].attrib["name"]])

            nonfixed_child_link_names_list_lists.append(nonfixed_child_link_names_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_child_link_names_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_child_link_names_list_lists.append([urdf_all_joints[i].find("child").attrib["link"]])

            nonfixed_joint_nums_list_lists.append(nonfixed_joint_nums_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_nums_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_nums_list_lists.append([i-fixed_joint_cnt])

            nonfixed_joint_unit_vectors_list_lists.append(nonfixed_joint_unit_vectors_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_unit_vectors_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_unit_vectors_list_lists.append([(urdf_all_joints[i].find("axis").attrib["xyz"])])

            nonfixed_joint_link_lengths_list_lists.append(nonfixed_joint_link_lengths_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_link_lengths_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_link_lengths_list_lists.append([(urdf_all_joints[i].find("origin").attrib["xyz"])])

            nonfixed_joint_postures_list_lists.append(nonfixed_joint_postures_list_lists[j][k+1-fixed_joint_cnt_in_tree:])
            del nonfixed_joint_postures_list_lists[-2][k+1-fixed_joint_cnt_in_tree:]
            nonfixed_joint_postures_list_lists.append([(urdf_all_joints[i].find("origin").attrib["rpy"])])

          break


# 初めに読んだlinkが他末端linkのchildかもしれない処理
# TODO: 古い状態で不備あり。j, kの2重ループに更新するべき。nonfixedも追加するべき。
parent_link_name = joint_obj_list_lists[1][0].find("parent").attrib["link"]
#print(parent_link_name)
for j in range(0, len(child_link_names_list_lists)):

  if parent_link_name == child_link_names_list_lists[j][-1]:
    joint_names_list_lists[j] = joint_names_list_lists[j]+joint_names_list_lists[1]
    child_link_names_list_lists[j] = child_link_names_list_lists[j]+child_link_names_list_lists[1]
    joint_nums_list_lists[j] = joint_nums_list_lists[j]+joint_nums_list_lists[1]


# [0]にある[None]を削除
joint_names_list_lists.pop(0)
joint_nums_list_lists.pop(0)
joint_obj_list_lists.pop(0)
child_link_names_list_lists.pop(0)
joint_link_lengths_list_lists.pop(0)
joint_postures_list_lists.pop(0)
nonfixed_joint_names_list_lists.pop(0)
nonfixed_joint_nums_list_lists.pop(0)
nonfixed_joint_obj_list_lists.pop(0)
nonfixed_child_link_names_list_lists.pop(0)
nonfixed_joint_unit_vectors_list_lists.pop(0)
nonfixed_joint_link_lengths_list_lists.pop(0)
nonfixed_joint_postures_list_lists.pop(0)
# 空欄listを削除
# nonfixed_joint_names_list_lists = [nonfixed_list for nonfixed_list in nonfixed_joint_names_list_lists if nonfixed_list != []]
# nonfixed_joint_nums_list_lists = [nonfixed_list for nonfixed_list in nonfixed_joint_nums_list_lists if nonfixed_list != []]
# nonfixed_joint_obj_list_lists = [nonfixed_list for nonfixed_list in nonfixed_joint_obj_list_lists if nonfixed_list != []]
# nonfixed_child_link_names_list_lists = [nonfixed_list for nonfixed_list in nonfixed_child_link_names_list_lists if nonfixed_list != []]


# userによる選択

# 各treeを表示
print("[INFO] List of tree in URDF file. (with fixed joints)")
for i in range(0, len(joint_names_list_lists)):
  print("   " + "tree " + str(i) + " : " + str(joint_names_list_lists[i]))

print("")

i = 0
joint_group_names_list = []
joint_names_group_dicts_list = []
nonfixed_joint_names_group_dicts_list = []
joints_poss_name = [all_joints_name[i]+"_sensor" if i not in fixed_joints_num else None for i in range(0, len(all_joints_name))]  # fixedな関節にはNone
nonfixed_joints_poss_name = [name+"_sensor" for name in nonfixed_joints_name]  # PositionSensor名list. URDFになくてProtoにはある。Joint名の語尾に"_sensor"が付く。

print("[INFO] Please input free joint_tree_name. \n" + \
    "[INFO] If nothing is inputted, joint_tree_name is 'names_group_<number>'.")
# treeごとに名前を要求
while i < len(joint_names_list_lists):
  print("[INFO] tree " + str(i) + " name?", end=" > ")
  joint_names_group = input().strip().replace(" ","_")  # userからのinput（先頭・末尾の" "を無視、" "を"_"に置き換え）

  # 未入力の場合
  if joint_names_group == "":
    joint_names_group = "names_group_"+str(i)

  joint_group_names_list.append(joint_names_group)

  # YAML用のdictを作成
  joint_names_group_dicts_list.append({joint_names_group: {
                                          "joint_names": joint_names_list_lists[i],
                                          # "joint_numbers": joint_nums_list_lists[i],
                                          "link_length": joint_link_lengths_list_lists[i],
                                          "joint_posture": joint_postures_list_lists[i]
                                        }
                                      })
  nonfixed_joint_names_group_dicts_list.append({joint_names_group: {
                                          "joint_names": nonfixed_joint_names_list_lists[i].append("null") if nonfixed_joint_names_list_lists[i]==[] else nonfixed_joint_names_list_lists[i],
                                          "position_sensor_names": "null" if nonfixed_joint_names_list_lists[i]==["null"] else [name+"_sensor" for name in nonfixed_joint_names_list_lists[i]],
                                          "joint_numbers": nonfixed_joint_nums_list_lists[i].append("null") if nonfixed_joint_nums_list_lists[i]==[] else nonfixed_joint_nums_list_lists[i],
                                          "joint_unit_vector": nonfixed_joint_unit_vectors_list_lists[i].append("null") if nonfixed_joint_unit_vectors_list_lists[i]==[] else nonfixed_joint_unit_vectors_list_lists[i],
                                          "link_length": nonfixed_joint_link_lengths_list_lists[i].append("null") if nonfixed_joint_link_lengths_list_lists[i]== [] else nonfixed_joint_link_lengths_list_lists[i],
                                          "joint_posture": nonfixed_joint_postures_list_lists[i].append("null") if nonfixed_joint_postures_list_lists[i]== [] else nonfixed_joint_postures_list_lists[i]
                                        }
                                      })
  i = i + 1
# pprint.pprint(joint_group_names_list)
print("")


# name's number | joint_group_name | joint_group_value の出力
print("name's number | joint_group_name | joint_group_value")
print("--------------|------------------|------------------")
for i in range(0, len(joint_names_group_dicts_list)):
  print("      " + str(i) + "       |  " + joint_group_names_list[i] + "   | " + str(joint_names_group_dicts_list[i][joint_group_names_list[i]]["joint_names"]))
print("")

# limb_key_listに対応するjoint_groupを取得
limb_key_list = ["left_leg", "right_leg"]
limb_name_list = {}
for i in range(0, len(limb_key_list)):

  while True:
    print("[INFO] What's " + limb_key_list[i] + "'s name? (please input name's number)", end=" > ")
    group_names_num = input().strip().replace(" ","")

    if group_names_num == "q":
      quit()
    #elif group_names_num < 0 or group_names_num > len(joint_group_names_list):
    elif not group_names_num.isascii() or not group_names_num.isdecimal() or int(group_names_num) > len(joint_group_names_list):
      print("[ERROR] Please input name's number!")
    else:

      limb_name_list[limb_key_list[i]] = joint_group_names_list[int(group_names_num)]
      break
# print(limb_name_list)


# jointごとのParameterを取得
all_joints_origin = []
all_joints_limit = []
all_joints_axis = []

# URDFから必要パラメータの取得
for num_joint in range(0, len(urdf_all_joints)):

  # joint単体リンク取得
  urdf_joint = urdf_all_joints[num_joint]  

  # origin xyz
  joint_ori_xyz = []
  try:
    str_vec = urdf_joint.find("origin").attrib["xyz"]
    joint_ori_xyz = (str_vec)
  except:
    print("[WARNING][" + urdf_joint.attrib["name"] + "] Origin_xyz_tag is not defined.")
    joint_ori_xyz.append("0.0, 0.0, 0.0")

  # origin rpy
  joint_ori_rpy = []
  try:
    str_vec = urdf_joint.find("origin").attrib["rpy"]
    joint_ori_rpy = (str_vec)
  except:
    print("[WARNING][" + urdf_joint.attrib["name"] + "] Origin_rpy_tag is not defined.")
    joint_ori_rpy.append("0.0, 0.0, 0.0")

  # add 
  origin = []
  origin.append(joint_ori_xyz)
  origin.append(joint_ori_rpy)
  all_joints_origin.append(origin)

  # axis xyz
  joint_axis = []
  try:
    str_vec = urdf_joint.find("axis").attrib["xyz"]
    joint_axis = (str_vec)
  except:
    print("[WARNING][" + urdf_joint.attrib["name"] + "] Axis_tag is not defined.")
    joint_axis.append(None)

  # add
  all_joints_axis.append(joint_axis)

  # limit
  try:
    joint_lim_eff = float(urdf_joint.find("limit").attrib["effort"])
  except:
    joint_lim_eff = None
  #
  try:
    joint_lim_low = float(urdf_joint.find("limit").attrib["lower"])
  except:
    joint_lim_low = None
  #
  try:
    joint_lim_upp = float(urdf_joint.find("limit").attrib["upper"])
  except:
    joint_lim_upp = None
  #
  try:
    joint_lim_vel = float(urdf_joint.find("limit").attrib["velocity"])
  except:
    joint_lim_vel = None

  # add
  limit = []
  limit.append(joint_lim_eff)
  limit.append(joint_lim_low)
  limit.append(joint_lim_upp)
  limit.append(joint_lim_vel)
  all_joints_limit.append(limit)


# YAMLに書き込むDictの構築（Joint）
joints_dict = {}
for num_joint in range(0, len(urdf_all_joints)):
  dict_origin = {
    "xyz": all_joints_origin[num_joint][0],
    "rpy": all_joints_origin[num_joint][1]
  }
  if num_joint not in fixed_joints_num:
    dict_limit = {
      "effort": all_joints_limit[num_joint][0],
      "lower": all_joints_limit[num_joint][1],
      "upper": all_joints_limit[num_joint][2],
      "velocity": all_joints_limit[num_joint][3]
    }
    joint_status = {
      "position_sensor_name": joints_poss_name[num_joint],
      "origin": dict_origin,
      "axis": all_joints_axis[num_joint],
      "limit": dict_limit
    }
  else:
    joint_status = {
      "origin": dict_origin
    }
  joints_dict[all_joints_name[num_joint]] = joint_status


# YAMLに書き込むDictの構築（Joint_Name & PosSensor_Name）
all_names = {
  "all_joint_names": all_joints_name,
  "all_position_sensor_names": joints_poss_name
}
# for joint_names_group_dict in joint_names_group_dicts_list:  # limbごとにわける処理

all_nonfixed_names = {
  "all_joint_names": nonfixed_joints_name,
  "all_position_sensor_names": nonfixed_joints_poss_name
}
# for nonfixed_joint_names_group_dict in nonfixed_joint_names_group_dicts_list:  # limbごとにわける処理
#   all_nonfixed_names = all_nonfixed_names | nonfixed_joint_names_group_dict

names_dict = {
  "all_names_without_fixed_joints": all_nonfixed_names,
  "all_names_with_fixed_joints": all_names
}


# YAMLに書き込むDictの構築（limb）
limb_with_fixed = {}
limb_without_fixed = {}

for joint_names_group_dict in joint_names_group_dicts_list:
  limb_with_fixed = limb_with_fixed | joint_names_group_dict
for nonfixed_joint_names_group_dict in nonfixed_joint_names_group_dicts_list:
  limb_without_fixed = limb_without_fixed | nonfixed_joint_names_group_dict

limb_dict = {
  "limb_names": limb_name_list,
  "limb_without_fixed_joints": limb_without_fixed,
  "limb_with_fixed_joints": limb_with_fixed
}

# YAMLに書き込むDict
dict_yaml_name_lists = {
  "/**": {
    "ros__parameters": {
      robot_name+"_name_lists": names_dict
    }
  }
}
dict_yaml_joints = {
  "/**": {
    "ros__parameters": {
      robot_name+"_joints": joints_dict
    }
  }
}
dict_yaml_limb = {
  "/**": {
    "ros__parameters": {
      robot_name+"_limb": limb_dict
    }
  }
}

# YAMLへの書き込み
with open("../../robot_description/config/"+robot_name+"/param_"+robot_name+"_name_lists.yaml", "w") as yaml_f:
  # 先頭のcommentの書き込み
  yaml_f.write("# Source URDF file path : " + urdf_path + 
"""
# Generated by robot_tools/robot_tools/model_parameter_generator.py

# name_lists

""")
  # parameterの書き込み
  yaml.safe_dump(
    dict_yaml_name_lists,
    yaml_f,
    default_flow_style=False,
    sort_keys=False
  )
with open("../../robot_description/config/"+robot_name+"/param_"+robot_name+"_joints.yaml", "w") as yaml_f:
  # 先頭のcommentの書き込み
  yaml_f.write("# Source URDF file path : " + urdf_path + 
"""
# Generated by robot_tools/robot_tools/model_parameter_generator.py

# joints

""")
  # parameterの書き込み
  yaml.safe_dump(
    dict_yaml_joints,
    yaml_f,
    default_flow_style=False,
    sort_keys=False
  )
with open("../../robot_description/config/"+robot_name+"/param_"+robot_name+"_limb.yaml", "w") as yaml_f:
  # 先頭のcommentの書き込み
  yaml_f.write("# Source URDF file path : " + urdf_path + 
"""
# Generated by robot_tools/robot_tools/model_parameter_generator.py

# limb

""")
  # parameterの書き込み
  yaml.safe_dump(
    dict_yaml_limb,
    yaml_f,
    default_flow_style=False,
    sort_keys=False
  )
