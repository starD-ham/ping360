# 曲座標変換
python コード
参考：https://qiita.com/sai-sui/items/b80e64869fb44d7df604


import cv2
import numpy as np
import matplotlib.pyplot as plt

// 極座標変換座標を返す
// wid:画像サイズ
// ratio:??
def get_rt(wid, ratio):
i = np.tile(np.arange(wid).reshape(-1, 1), (1, wid)) - wid / 2 // 配列を並べる。0~wid-1の数値をもつ縦ベクトルを横に繰り返す。wid/2を引くことで
j = i.T
r = np.linalg.norm((i, j), axis=0)　// i,j の列ごとのベクトルノルムを取り、行ベクトルを求める
r /= np.max(r) //0~1で正規化
r *= (wid - 1) * ratio　// 0~(wid-1)*ratioで正規化
r[r >= wid] = wid - 1 // widを超えているrの要素をwid-1にする
j[j == 0] = 1 // j=0となっているjの要素を1にする
t = np.arctan(i / j)[::-1] //　それぞれの要素についてarctanを計算していき[::-1]後ろから一つずつとりだす
t[j < 0] += np.pi // j<0となっている要素番号のtにpiを足す
t -= np.min(t)　// 最小値を0とする
t /= np.max(t) // 0-1で正規化
t *= (wid - 1) // 0~wid-1で正規化
t[t >= wid] = wid - 1// widを超えているtの要素をwid-1にする
return r.astype('i'), t.astype('i')　// intへキャストして返す

//直行座標変換を行う関数
def get_xy(wid):
a = np.arange(wid) / wid
r = wid / 2
x = (a.reshape(1, -1) * np.cos(2 * a.reshape(-1, 1) * np.pi) * r + r).astype('i')
y = (a.reshape(1, -1) * np.sin(2 * a.reshape(-1, 1) * np.pi) * r + r).astype('i')
x[x < 0] = 0
x[x >= wid] = wid - 1
y[y < 0] = 0
y[y >= wid] = wid - 1
return x, y