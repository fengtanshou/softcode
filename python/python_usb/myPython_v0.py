#!/usr/bin/python
#coding=utf-8

import os
import sys
import time
import pickle as p
import threading

USERID = 1
TYPEID = 1
ACTIONID = 1
warning = 0
video_t = 0
files_t = ''

mutex=threading.Lock() 
f = open("./log.txt", 'w+') 
#---------------------------------------------------
#list start
class Node():
    def __init__(self,value = None,next = None):
        self._value = value 
        self._next = next
        print ("self._vaule = ",self._value, file = f)
        print ("self._next = ",self._next ,file = f)

class Chain():
    def __init__(self):
        self._head = None
        self.length = 0
    
    def isEmpty(self):
        return self.length == 0

    def append(self,value):
        if isinstance(value,Node):
            node = value
        else:
            node = Node(value)

        if self._head == None:
            self._head = node
        else:
            be_node = self._head
            while be_node._next:
                be_node = be_node._next
            be_node._next = node
        self.length += 1

    def delete(self, index):
        print("index = ",index,file = f)
        if type(index) is int:
            if ( index > self.length ) or ( index <= 0 ):
                print("Index  is out of range.")
                return
            else:
                if index == 1:
                    self._head = self._head._next
                    self.length -=1
                else:
                    current_node = self._head
                    while index - 1:
                        current_node = current_node._next
                        index -= 1
                    current_node = current_node._next
                    self.length -= 1
                    return

        else:
            print("Index value is not int.")
            return
    def printlist(self):       # 打印链表  
        p=self._head  
        print (self.length)
        while p!=None:  
             print (p._value)  
             p=p._next  

    def getlength(self):
        return int(self.length)

    def gethead(self):
        return self._head
#list end

chain = Chain()

##判断记录是否存在，不存在则创建记录
'''
if os.path.exists("luozh.dat")==False:
    f=open("luozh.dat","wb")
    temp={'total':0}
    p.dump(temp,f)
    f.close()
    print ("\n本地磁盘尚无存储记录文件，新记录文件创建成功！\n")

    f=open("luozh.dat",'rb')
    list=p.load(f)
    print("list=",list)
    f.close()
else:
    pass
'''
#添加测试人员
def add():
    f=open('luozh.dat','rb')
    a=p.load(f)
    #debug
    print("a=",a) 
    f.close()
    userid=input("请输入要添加的测试人的UserID:")
    if userid in a:
        print ("\nUserId已经存在，添加失败，请确认！")
    else:
        typeid=input('请输入配饰ID：')
        action=input('请输入采集动作ID:')
        list=[typeid,action]
        information={userid:list}
        a['total']+=1
        print("a=",a)
        a.update(information)
        f=open('luozh.dat','wb')
        p.dump(a,f)
        f.close()
        print("添加成功\n")

#显示测试人员
def showall():
    f=open('luozh.dat','rb')
    a=p.load(f)
    print("一共有{}个测试人员.\n".format(a['total']))
    print("{:<8}\t{:<8}\t{:<8}".format('人员ID','配饰ID','动作ID'))
    for key in a.keys():
        if key!='total':
            print("{:<8}\t{:<15}\t{:<30}".format(key,a[key][0],a[key][1]))

    f.close()

#推出程序
def exit():
    gstserver="ps -aux | grep gst-launch-1.0 | awk '{print $2}'"
    ifserver="ps -aux | grep ifconfig | awk '{print $2}'"
    result = os.popen(gstserver)
    res = result.read()
    if res != '':
        for line in res.splitlines():
            kill = "kill %s"%line
            os.system(kill)

    result = os.popen(ifserver)
    res = result.read()
    if res != '':
        for line in res.splitlines():
            kill = "kill %s"%line
            os.system(kill)
    video_clear = "./sender -t 1 > /dev/null 2>&1 & "
    if os.system(video_clear):
        print ("video clear failed")
    f.close()
    sys.exit()

#查找
def search(userid):
    f=open('luozh.dat','rb')
    a=p.load(f)
    if userid in a:
        print("{}的配饰ID是:{}".format(userid,a[userid][0]))
        print("{}的动作ID是:{}".format(userid,a[userid][1]))
    else:
        print("userid不存在！\n")
    f.close()

#删除
def delete(userid):
    f=open("luozh.dat","rb")
    a=p.load(f)
    f.close()
    if userid in a:
        a.pop(userid)
        a['total']-=1
        f=open('luozh.dat','wb')
        p.dump(a,f)
        f.close()
        print("删除成功!\n")
    else:
        print("user不存在!\n")


#修改
def modify(userid):
    f=open("luozh.dat","rb")
    a=p.load(f)
    f.close()
    if userid in a:
        typeid=input("请输入修改后的配饰ID:")
        a[userid][0]=typeid
        actionid=input("请输入修改后的动作ID:")
        a[userid][1]=actionid
        f=open('luozh.dat','wb')
        p.dump(a,f)
        f.close()
        print("修改成功！\n")
    else:
        print("user不存在!\n")


def interfaceup():
    interface = "ifconfig | grep enp0s20 | awk '{print $1}'"
    result = os.popen(interface)
    res=result.read()
    if res != '':
        for line in res.splitlines():
            interface_1 = "sudo ifconfig '%s' 192.168.1.10 up  > /dev/null 2>&1 &"%line
            if os.system(interface_1) != 0:
                print ("ifconfig failed")
            time.sleep(2)
    
def adjust():
    result = '''ssh -f -n root@192.168.1.2 "echo 0x4300 >/sys/devices/platform/11010000.i2c/i2c-3/3-0068/register_addr" '''
    result_t = ''' ssh -f -n root@192.168.1.2 "echo 0x3a >/sys/devices/platform/11010000.i2c/i2c-3/3-0068/sensor_register_ov10635" '''
    os.system(result)
    time.sleep(0.01)
    os.system(result_t)

def loop():
    while True:
        time.sleep(10)
        interfaceup()
    
def PrepareRecord():
    camera='''ssh root@192.168.1.2 "ls -l /sys/class/video4linux | grep 10217000.mipicsi" | awk '{print $9}' '''
    #camera='''ssh root@192.168.1.2 "ls -l /sys/class/video4linux | grep 10218000.mipicsi" | awk '{print $9}' '''
    cam_result=os.popen(camera)
    cam_res=cam_result.read()
    for camDev in cam_res.splitlines():
        #preview
        gladuis_sub = "gst-launch-1.0 v4l2src device='/dev/%s'  ! videoconvert ! jpegenc !  tcpserversink port=8554 host=0.0.0.0"%camDev
        gladuis = "ssh -f -n root@192.168.1.2 " + '\"' +  gladuis_sub + '\"' + "> /dev/null 2>&1 &"
        #print (gladuis)
        #save yuv data
        #gladuis_save = "gst-launch-1.0 v4l2src device='/dev/%s' !  tcpserversink port=8554 host=0.0.0.0"%camDev
        #gladuis_save_cmd = "ssh -f -n root@192.168.1.2 " + '\"' +  gladuis_save + '\"' + "> /dev/null 2>&1 &"

    #1 check gst-launch-1.0 process
    gstreamer = '''ssh root@192.168.1.2 "ps -A | grep "gst-launch-1.0"" | awk '{print $1}' '''
    gstr_result = os.popen(gstreamer)
    gstr_res = gstr_result.read()
    for gstr_line in gstr_res.splitlines():
        killpid = "kill %s"%gstr_line
        kill = "ssh root@192.168.1.2 " + '\"' + killpid + '\"'
        os.popen(kill)
    #preview
    if os.system(gladuis) != 0:
        print("gladuis prebiew fail")
    #save
    #if os.system(gladuis_save_cmd) !=0:
    #    print("gladuis save cmd fail")

def prepare():
    video = '''ssh root@192.168.1.2 "ps -A | grep videoRecoder" | awk '{print $1}' '''
    result = os.popen(video)
    res = result.read()
    if res != '':
        for line in res.splitlines():
            killpid = "kill %s"%line
            kill = "ssh root@192.168.1.2 " + '\"' + killpid + '\"'
            os.popen(kill)
    #-----------------------------
    videosrc = '''ssh root@192.168.1.2 "ls /flash | grep video.data" '''
    result = os.popen(videosrc)
    res = result.read()
    if res != '':
        for line in res.splitlines():
            src = "rm /flash/%s"%line
            rm_src = "ssh root@192.168.1.2 " + '\"' + src + '\"'
            os.popen(rm_src)
    #-----------------------------
    ssh = "scp -r videoRecoder  root@192.168.1.2:/flash > /dev/null 2>&1 &"
    if os.system(ssh) !=0:
        print ("ssh scp failed")
    time.sleep(2)
    result = '''ssh -f -n root@192.168.1.2 "/flash/videoRecoder -p 8554 -e 800 -w 1280 -s 1 " > /dev/null 2>&1 & '''
    if os.system(result) !=0:
        print("/flash/videoRecoder failed")


#preview
def startpriview():    
    #preview
    gladuis_pc="gst-launch-1.0 tcpclientsrc  port=8554 host=192.168.1.2 ! jpegdec ! videoconvert ! autovideosink > /dev/null 2>&1 &"
    if os.system(gladuis_pc) != 0:
        print("gladuis_pc command fail",file = f)

def startRecord():
    global USERID , TYPEID , ACTIONID , warning ,files_t
    if chain.getlength() > 3:
        print ("\n4 files is recordding, waiting a few minutes!")
        return
    print ("用户ID:配饰ID:动作ID   (%d:%d:%d)"%(USERID,TYPEID,ACTIONID))
    while True:
        x = input("是否是所需要参数:y/n:")
        if x == 'y':
            print ("\nrecording ...")
            break
        elif x == 'n':
            USERID = int(input("输入userID:")) 
            TYPEID = int(input("输入配饰ID:"))
            ACTIONID = int(input("输入动作ID:"))
            print ("用户ID:配饰ID:动作ID   (%d:%d:%d)"%(USERID,TYPEID,ACTIONID))
            if USERID > 2500 or USERID < 1 or TYPEID > 5 or TYPEID < 1 or ACTIONID > 20 or ACTIONID < 1 : 
                print ("数据不在测试所需范围内,请确认!")
                continue
            break
        else:
            continue
    #start record
    files_t = "%s_%s_%s_"%(USERID,TYPEID,ACTIONID) + time.strftime('%Y%m%d%H%M') + ".data"
    video_save = "./sender -a 1 -f /flash/%s  > /dev/null 2>&1 &"%files_t
    if os.system(video_save) != 0:
        print ("video_save failed")
    warning = 1
    print ("Recording ...")
    
def checkdisk():
    harddisk = "df -h"
    if os.system(harddisk) != 0:
        print ("check disk error!")

def loop_scp():
    global video_t
    while True:
        time.sleep(0.3)
        if chain.getlength() >= 1 and chain.getlength() <= 3 and chain.gethead() != None:
            mutex.acquire()
            size_t = ''' ssh -f -n root@192.168.1.2 "ls -l /flash/%s"  | awk '{print $5}' '''%chain.gethead()._value
            print (size_t,file = f)
            result = os.popen(size_t)
            res = result.read()
            if res != '':
                for line in res.splitlines():
                    print ("gladuis line = %s"%line,file = f)
                    video_t = line
            print ("copy ...",file=f)
            #scp_t = "scp -r -C root@192.168.1.2:/flash/video%d.data  "%chain.getlength() + "./%s_%s_%s_"%(USERID,TYPEID,ACTIONID) + \
            #time.strftime('%Y%m%d%H%M') + ".data > /dev/null 2>&1 &"
            scp_t = "scp -r -C root@192.168.1.2:/flash/%s  "%chain.gethead()._value + " ./   > /dev/null 2>&1 &"
            if os.system(scp_t) != 0:
                print ("scp video.data failed")
            time.sleep(2)
            while True:
                time.sleep(3)
                file_size = "ls -l  %s | awk '{print $5}'"%chain.gethead()._value
                result = os.popen(file_size)
                res = result.read()
                if res != '':
                    for line in res.splitlines():
                        size = line
                        print ("pc line = %s"%line , file = f)
                else:
                    continue

                if video_t == size: 
                    break
                else:
                    continue
            #and remove file
            videosrc = '''ssh root@192.168.1.2 "ls /flash | grep %s" '''%chain.gethead()._value
            result = os.popen(videosrc)
            res = result.read()
            if res != '':
                for line in res.splitlines():
                    src = "rm /flash/%s"%line
                    rm_src = "ssh root@192.168.1.2 " + '\"' + src + '\"'
                    os.system(rm_src)
            #remoce end
            chain.delete(1)

            mutex.release()
            print ("copy over!" , file = f)
            continue
        elif chain.getlength() > 3:
            print ("\nbe copying , Please wait a few mimutes.")
            continue
        else:
            continue

def stoprecord():
    global USERID , TYPEID , ACTIONID ,warning , video_t,files_t
    if warning == 0:
        print ("\nnot start record ^-^")
        return
    print ("%d %d %d"%(USERID,TYPEID,ACTIONID))
    if USERID > 2500 or TYPEID > 5 or ACTIONID > 20 :
        print ("输入数据超出范围,请确认!")

    video_stop = "./sender -a 0 > /dev/null 2>&1 &"
    if os.system(video_stop) != 0:
        print ("video_stop faild")
    else: 
        chain.append(files_t)

    #update ID
    warning = 0
    if TYPEID == 5 and ACTIONID == 20:
        USERID = USERID + 1
        TYPEID = 1
        ACTIONID = 1
    if ACTIONID == 20:
       TYPEID = TYPEID + 1
       ACTIONID = 1
    elif ACTIONID != 20:
       ACTIONID = ACTIONID + 1


def replay():
    userid = input("user_id:")
    typeid = input("配饰ID:")
    actionid = input("动作ID:")

    files = "%s_%s_%s*.data"%(userid,typeid,actionid)
    files_src = "ls %s"%files 
    print ("%s",files_src)
    result = os.popen(files_src)
    res = result.read()
    if res != '':
        for line in res.splitlines():
            replay = "gst-launch-1.0 filesrc location=%s ! videoparse width=1280 height=800 format=4 \
            framerate=15/1 ! videoconvert ! autovideosink > /dev/null 2>&1 &"%line
            os.system(replay)

#界面
def menu():
    print("                                       ")
    print("********************************************")
    print("       显示菜单提示信息：             *")
    print("       开启预览:                      1")
    print("       开始录制:                      2")
    print("       停止录制：                     3")
    print("       回看播录视频                   4")
    print("       查看系统硬盘容量：             5")
    #print("       显示所有测试人员：             6")
    #print("       查找测试人员[user_id]：        7")
    #print("       添加测试人员[user_id]：        8")
    #print("       删除测试人员[user_id]：        9")
    #print("       更改测试人资料[user_id]：      10")
    print("       图像调整                       a")
    print("       推出程序：                     b")
    print("********************************************")
    '''
    print("\n***************************配饰****************************")
    print("*                                                         *")
    print("*  1:戴眼镜 2:不戴眼镜 3:戴墨镜 4:戴口罩 5:戴围巾和帽子   *")
    print("*                                                         *")
    print("***********************************************************\n")
    '''

interfaceup()


threads = []
t1 = threading.Thread(target = loop)
threads.append(t1)
t2 = threading.Thread(target = prepare)
threads.append(t2)
t3 = threading.Thread(target = loop_scp)
threads.append(t3)
if __name__ == '__main__':
    for t in threads:
        t.setDaemon(True)
        t.start()

#主程序
while True:
    if warning == 0:
        menu()
    x=input("\n请输入你的选择菜单号:")
    if warning == 1 and x != '3':
        print ("正在录制中，需要暂停后在进行其他步骤！！")
        continue
    if x=='1':
        startpriview()
        continue
    if x=='2':
        startRecord()
        continue
    if x=='3':
        stoprecord()
        continue
    if x=='4':
        replay()
        continue
    if x=='5':
        checkdisk()
        continue
    if x=='a':
        adjust()
        continue
    if x=='b':
        print("谢谢使用!\n")
        exit()
    '''    
    if x=='6':
        showall()
        continue
    if x=='7':
        user_id=input("请输入所要查找的测试人员user_id:")
        search(user_id)
        continue
    if x=='8':
        add()
        continue
    if x=='9':
        user_id=input("请输入所要删除的测试人员user_id:")
        delete(user_id)
        continue
    if x=='10':
        user_id=input("请输入所要修改的user_id:")
        modify(user_id)
        continue
    if x=='*':
        menu()
    '''



        



























