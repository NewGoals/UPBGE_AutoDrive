import bge
import socket
import _thread
import threading
import  time
from collections import OrderedDict
from bge import logic
import os.path
lock=threading.Lock()
图片_字节集=None
遮蔽=False
def 读取图片_到字节集(filename):
    #print('开始',time.time())
    buf = bytearray(os.path.getsize(filename))
    #print('开始1',time.time())
    with open(filename, 'rb') as f:
       # print('开始2',time.time())
        f.readinto(buf)
        #print('开始3',time.time())
    return buf

def 组包_总(综合信息,图片_字节集):
    综合信息_文本=str(综合信息)
    综合信息_字节集=综合信息_文本.encode()
    类型='综'.encode('utf8')
    临=类型+综合信息_字节集
    长度_字节集 = (len(临)+4).to_bytes(4, byteorder='big')
    包_综合=长度_字节集+临
    类型 = '图'.encode('utf8')
    临 = 类型 + 图片_字节集
    长度_字节集 = (len(临)+4).to_bytes(4, byteorder='big')
    包_图=长度_字节集+临
    长度 = int().from_bytes(长度_字节集, byteorder='big', signed=True)
    临=包_综合+包_图
    长度_字节集 = (len(临) + 4).to_bytes(4, byteorder='big')
    包_汇总=长度_字节集+临
    return 包_汇总

计数=0
def 启动TCP服务():
    
    global 图片_字节集,遮蔽,计数
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # 监听端口
    s.bind(('127.0.0.1', 6666))
    
    # 调用listen()方法开始监听端口，传入的参数指定等待连接的最大数量
    s.listen(1)
    保持TCP服务=True
    
    while 保持TCP服务:
        sock, addr = s.accept()
        #buf = sock.recv(1024).decode()
        while True:
           

            #print('开始1',time.time())  
            try :
                data =str(全局字典['碰撞次数'])
            except:
                s.close()
                保持TCP服务=False
                break
            #print('开始2',time.time())  
            buf = sock.recv(1024).decode()
            #print('开始3',time.time()) 
            if buf!='':
                TCP客户端操控字典 = eval(buf)
  
                #print(type(user_dict),user_dict)
                if buf == 'exit':
                    s.close()
                    保持TCP服务=False
                    break
                    
                try :
                    待发送数据={}
                    待发送数据['载具坐标']=全局字典['载具坐标']
                    待发送数据['碰撞次数']=全局字典['碰撞次数']

                    待发送数据['载具方向角']=全局字典['载具方向角']
                    待发送数据['载具方向角x']=全局字典['载具方向角x']
                    待发送数据['载具方向角y']=全局字典['载具方向角y']
                    待发送数据['载具速度']=全局字典['载具速度']
                    待发送数据['轮向角']=全局字典['轮向角']
                    data =str(待发送数据)
                except:
                    s.close()
                    保持TCP服务=False
                    break
                
                全局字典['W按下']=TCP客户端操控字典['W按下'] 
                全局字典['S按下']=TCP客户端操控字典['S按下'] 
                全局字典['A按下']=TCP客户端操控字典['A按下'] 
                全局字典['D按下']=TCP客户端操控字典['D按下']                 
                全局字典['J按下']=TCP客户端操控字典['J按下'] 
                全局字典['K按下']=TCP客户端操控字典['K按下'] 
                全局字典['L按下']=TCP客户端操控字典['L按下'] 
                全局字典['M按下']=TCP客户端操控字典['M按下']
                全局字典['载具新坐标'].x=TCP客户端操控字典['起x']
                全局字典['载具新坐标'].y=TCP客户端操控字典['起y']
                全局字典['欧拉角Z']=TCP客户端操控字典['欧拉角Z']
                全局字典['P按下']=TCP客户端操控字典['P按下']
                
                #print(TCP客户端操控字典)
                if 图片_字节集==None:
                    #print('开始4',time.time()) 
                    
                    #print('开始5',time.time()) 
                    #bge.render.makeScreenshot('/home/fengquanli/图片/截屏{}.png'.format(str(1)))
                    lock.acquire()
                    遮蔽=True
                    if 计数<3:
                        取值=100-3+计数
                    else:
                        取值=计数-3


                    图片_字节集1 = 读取图片_到字节集('/home/fengquanli/NewDisk/python/图片/截屏{}.png'.format(str(取值)))
                    遮蔽=False
                    lock.release()
                    组包结果=组包_总(待发送数据,图片_字节集1)
                    
                    sock.send(组包结果)
                    #print('开始6',time.time())
            else:
                print("客户端断开连接。")
                break
        
        time.sleep(0.01)

def 截屏():
    global 计数,图片_字节集,遮蔽
    while True:
        #print('开始',time.time())
        if 计数==101:
            计数=0

        if 遮蔽==False:
            bge.render.makeScreenshot('/home/fengquanli/NewDisk/python/图片/截屏{}.png'.format(str(计数)))
            计数=计数+1
        
        # lock.acquire()
        # 图片_字节集 = 读取图片_到字节集('/home/fengquanli/图片/截屏{}.png'.format(str(计数)))
        try:
            if 计数<3:
                取值=100-3+计数
            else:
                取值=计数-3

            lock.acquire()
            图片_字节集 = 读取图片_到字节集('/home/fengquanli/NewDisk/python/图片/截屏{}.png'.format(str(取值)))
            lock.release()
        
        
        time.sleep(0.033)
        #print('开始1',time.time())
        try :

            data =str(全局字典['碰撞次数'])
        except:
            break



全局字典 = bge.logic.globalDict #Get the global dictionary
try:
    关残留socket进程 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    关残留socket进程.connect(('127.0.0.1', 6666))
    data='exit'
    关残留socket进程.send(data.encode())
except:
   print ("无残留进程")
#print(dir(bge.logic))

class pengzhuang(bge.types.KX_PythonComponent):

    args = OrderedDict([
    ])

    def start(self, args):
        全局字典['碰撞次数']= 0
        if self.object.name=='碰撞树':
            t = threading.Thread(target=启动TCP服务)
            t.start()
            截=threading.Thread(target=截屏)
            截.start()


        self.object.debug=True
        self.object.collisionCallbacks.append(self.on_collision_one)

        

        


      

    def update(self):

        scene = self.object.scene

            
    def on_collision_one(self, object):
        碰撞次数=全局字典['碰撞次数']
        碰撞次数=碰撞次数+1
        全局字典['碰撞次数']=碰撞次数
        print('碰撞次数-------------------------------------------',碰撞次数)
        print(全局字典['碰撞次数'],全局字典['位置'])

        print('{}Hit by {}'.format(self.object.name,object.name))  
