#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts')

from common_import import *


#--------------------------------------------------
#SLAMで移動するときに必要なパラメータ
#--------------------------------------------------
#PythonとC++どちらでも扱いやすいように、分けて宣言
rospy.set_param('/param/slam/goal/pos/x', 0.0) #float
rospy.set_param('/param/slam/goal/pos/y', 0.0) #float
rospy.set_param('/param/slam/goal/pos/yaw', 0.0) #float


#--------------------------------------------------
#iARMで把持する時に必要なパラメータ
#--------------------------------------------------
rospy.set_param('/param/iarm/obj/id', 0) #int 把持対象オブジェクトのid(オプション)
#PythonとC++どちらでも扱いやすいように、分けて宣言
rospy.set_param('/param/iarm/obj/pos/x', 0.0) #float 把持対象オブジェクトのbase_link座標系のx座標(必須)
rospy.set_param('/param/iarm/obj/pos/y', 0.0) #float 把持対象オブジェクトのbase_link座標系のy座標(必須)
rospy.set_param('/param/iarm/obj/pos/z', 0.0) #float 把持対象オブジェクトのbase_link座標系のz座標(必須)
rospy.set_param('/param/iarm/find/cnt', 0) #int
rospy.set_param('/param/iarm/find/try/cnt', 0) #int
rospy.set_param('/param/iarm/obj/height', 0.72) #float 把持対象オブジェクトが載っている板の高さ


#--------------------------------------------------
#オブジェクトのDB
#--------------------------------------------------
#rospy.set_param('/param/obj/db',
#[
#{'obj_id':1, 'obj_name_j':'グリーンティ', 'obj_name_e':'green tea', 'obj_class':'drink'},
#{'obj_id':2, 'obj_name_j':'オレンジジュース', 'obj_name_e':'orange juice', 'obj_class':'drink'},
#{'obj_id':3, 'obj_name_j':'ブラウンティ', 'obj_name_e':'brown tea', 'obj_class':'drink'},
#{'obj_id':4, 'obj_name_j':'ジャパニーズティ', 'obj_name_e':'japanese tea', 'obj_class':'drink'},
#{'obj_id':5, 'obj_name_j':'レッドティ', 'obj_name_e':'red tea', 'obj_class':'drink'},
#{'obj_id':6, 'obj_name_j':'レモンティ', 'obj_name_e':'lemon tea', 'obj_class':'drink'},
#{'obj_id':7, 'obj_name_j':'ストロベリージュース', 'obj_name_e':'strawberry juice', 'obj_class':'drink'},
#{'obj_id':8, 'obj_name_j':'カップスター', 'obj_name_e':'cup star', 'obj_class':'food'},
#{'obj_id':9, 'obj_name_j':'カップヌードル', 'obj_name_e':'cup noodle', 'obj_class':'food'},
#{'obj_id':10, 'obj_name_j':'シーフードヌードル', 'obj_name_e':'seafood noodle', 'obj_class':'food'},
#{'obj_id':11, 'obj_name_j':'コリアンスープ', 'obj_name_e':'korean soup', 'obj_class':'food'},
#{'obj_id':12, 'obj_name_j':'エッグスープ', 'obj_name_e':'egg soup', 'obj_class':'food'},
#{'obj_id':13, 'obj_name_j':'オニオンドレッシング', 'obj_name_e':'onion dressing', 'obj_class':'food'},
#{'obj_id':14, 'obj_name_j':'ジャパニーズドレッシング', 'obj_name_e':'japanese dressing', 'obj_class':'food'},
#{'obj_id':15, 'obj_name_j':'チップスター', 'obj_name_e':'chip star', 'obj_class':'snack'},
#{'obj_id':16, 'obj_name_j':'プリングルス', 'obj_name_e':'pringles', 'obj_class':'snack'},
#{'obj_id':17, 'obj_name_j':'ロングポテト', 'obj_name_e':'long potato', 'obj_class':'snack'},
#{'obj_id':18, 'obj_name_j':'ブルーポテト', 'obj_name_e':'blue potato', 'obj_class':'snack'},
#{'obj_id':19, 'obj_name_j':'レッドポテト', 'obj_name_e':'red potato', 'obj_class':'snack'},
#{'obj_id':20, 'obj_name_j':'スティックポテト', 'obj_name_e':'stick potato', 'obj_class':'snack'},
#{'obj_id':21, 'obj_name_j':'ブリーチ', 'obj_name_e':'bleach', 'obj_class':'cleaner'},
#{'obj_id':22, 'obj_name_j':'クロスクリーナー', 'obj_name_e':'cloth cleaner', 'obj_class':'cleaner'},
#{'obj_id':23, 'obj_name_j':'ソフナー', 'obj_name_e':'softener', 'obj_class':'cleaner'},
#{'obj_id':24, 'obj_name_j':'ディッシュクリーナー', 'obj_name_e':'dish cleaner', 'obj_class':'cleaner'},
#{'obj_id':25, 'obj_name_j':'バスクリーナー', 'obj_name_e':'bath cleaner', 'obj_class':'cleaner'},
#]
#)

rospy.set_param('/param/obj/db',
[
{'obj_id':1, 'obj_name_j':'グリーンティ', 'obj_name_e':'Green tea', 'obj_class':'drink'},
{'obj_id':2, 'obj_name_j':'カフェオレ', 'obj_name_e':'Cafe au lait', 'obj_class':'drink'},
{'obj_id':3, 'obj_name_j':'アイスティ', 'obj_name_e':'Iced tea', 'obj_class':'drink'},
{'obj_id':4, 'obj_name_j':'オレンジジュース', 'obj_name_e':'Orange juice', 'obj_class':'drink'},
{'obj_id':5, 'obj_name_j':'ストロベリージュース', 'obj_name_e':'Strawberry juice', 'obj_class':'drink'},
{'obj_id':6, 'obj_name_j':'ポテトチップス', 'obj_name_e':'Potato chips', 'obj_class':'snack'},
{'obj_id':7, 'obj_name_j':'クッキー', 'obj_name_e':'Cookie', 'obj_class':'snack'},
{'obj_id':8, 'obj_name_j':'ポテトスティック', 'obj_name_e':'Potato stick', 'obj_class':'snack'},
{'obj_id':9, 'obj_name_j':'ポタージュスープ', 'obj_name_e':'Potage soup', 'obj_class':'soup'},
{'obj_id':10, 'obj_name_j':'エッグスープ', 'obj_name_e':'Egg soup', 'obj_class':'soup'},
{'obj_id':11, 'obj_name_j':'オレンジ', 'obj_name_e':'Orange', 'obj_class':'fruit'},
{'obj_id':12, 'obj_name_j':'アップル', 'obj_name_e':'Apple', 'obj_class':'fruit'},
{'obj_id':13, 'obj_name_j':'ボウル', 'obj_name_e':'Bawl', 'obj_class':'container'},
{'obj_id':14, 'obj_name_j':'トレイ', 'obj_name_e':'Tray', 'obj_class':'container'},
{'obj_id':15, 'obj_name_j':'カップ', 'obj_name_e':'Cup', 'obj_class':'container'},

{'obj_id':16, 'obj_name_j':'プレモル', 'obj_name_e':'', 'obj_class':'drink'},
{'obj_id':17, 'obj_name_j':'ノンアルコールビール', 'obj_name_e':'', 'obj_class':'drink'},
{'obj_id':18, 'obj_name_j':'のどごしなま', 'obj_name_e':'', 'obj_class':'drink'},
{'obj_id':19, 'obj_name_j':'一番搾り', 'obj_name_e':'', 'obj_class':'drink'},
{'obj_id':20, 'obj_name_j':'スーパードライ', 'obj_name_e':'', 'obj_class':'drink'},
]
)
