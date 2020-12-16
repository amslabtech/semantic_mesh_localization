import numpy as np
import scipy.stats as sstats
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import seaborn as sns
import cv2

"%matplotlib inline"

def grayhist(img, stats=False):
    """
    入力 : BGR画像, 統計量の表示の有無(Optional)
    出力 : グレースケールのimshow, Histgram(+統計量)
    """
    # スタイルの設定。seabornの設定は残るため、常に最初に書いておく
    sns.set()
    sns.set_style(style='ticks')

    # プロット全体と個々のグラフのインスタンス
    fig = plt.figure(figsize=[15,4])
    ax1 = fig.add_subplot(1,2,1)
    
    sns.set_style(style='whitegrid')
    ax2 = fig.add_subplot(1,2,2)
    
    # グレースケール画像化→三重配列に戻す
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    ax1.imshow(img)
    
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # 一次元配列化
    img = np.array(img).flatten()
    # 本来rangeは[0,255]となるはずだが、255に値が集まりすぎたので弾いた
    img = img[img!=255]

    # 軸の刻み目
    ax2.set_xticks(np.linspace(0,255,9).astype(int))
    
    # ヒストグラムを計算→色をつける
    N, bins, patches = ax2.hist(img, range=(0,255), bins=256)
    for (i,patch) in enumerate(patches):
        color = cm.gray(bins[i]/256)
        patch.set_facecolor(color)

    if stats==True: # 統計量を表示する
        mean = img.mean()
        std = np.std(img)
        median = np.median(img)
        mode = sstats.mode(img)[0][0]

        # 統計量のラインをひく
        ax2.axvline(mean, color='#d95763', linestyle='solid', linewidth=3)
        ax2.axvline(median, color='#6abe30', linestyle='solid', linewidth=2)
        ax2.axvline(mode, color='#ba9641', linestyle='solid', linewidth=2)
        ax2.axvline(mean + std, color='#d95763', linestyle='dashdot', linewidth=1)
        ax2.axvline(mean - std, color='#d95763', linestyle='dashdot', linewidth=1)

        # 統計量の説明の文字
        ax2.text(mean,N.max()*1.075, "$\mu$",color='#d95763',horizontalalignment='center')
        ax2.text(median,N.max()*1.18,"median", color='#6abe30',rotation=45)
        ax2.text(mode,N.max()*1.15,"mode",color='#ba9641',rotation=45)
        ax2.text(mean+std,N.max()*1.075, "$\mu+\sigma$", color='#d95763',horizontalalignment='center')
        ax2.text(mean-std,N.max()*1.075, "$\mu-\sigma$", color='#d95763',horizontalalignment='center')

        fig.tight_layout()
        plt.show()
        
        print("mean    : ", mean)
        print("stddev  : ", std)
        print("median  : ", median)
        print("mode    : ", mode)
        
    else:
        fig.tight_layout()
        plt.show()


def rgb_hist(rgb_img):
    sns.set()
    sns.set_style(style='ticks')
    fig = plt.figure(figsize=[15,4])
    ax1 = fig.add_subplot(1,2,1)
    sns.set_style(style='whitegrid')
    ax2 = fig.add_subplot(1,2,2)
    
    ax1.imshow(rgb_img)

    color=['r','g','b']

    for (i,col) in enumerate(color): # 各チャンネルのhist
        # cv2.calcHist([img], [channel], mask_img, [binsize], ranges)
        hist = cv2.calcHist([rgb_img], [i], None, [256], [0,256])
        # グラフの形が偏りすぎるので √ をとってみる
        hist = np.sqrt(hist)
        ax2.plot(hist,color=col)
        ax2.set_xlim([0,256])

    plt.show()



wiz = cv2.cvtColor(cv2.imread('meshmapimage.jpg',1), cv2.COLOR_BGR2RGB)
rgb_hist(wiz)
