import os
import sys

USERID = 1
TYPEID = 1
ACTIONID = 1
warning = 0
def start():
    global USERID , TYPEID , ACTIONID , warning
    print ("用户ID:配饰ID:动作ID   (%d:%d:%d)"%(USERID,TYPEID,ACTIONID))
    
    while True:
        x = input("是否是所需要参数:y/n:")
        if x == 'y':
            print ("recording ...")
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

    print ("record")

    warning = 1

def stop():
    global USERID , TYPEID , ACTIONID ,warning
    if USERID > 2500 or TYPEID > 5 or ACTIONID > 20 :
        print ("输入数据超出范围,请确认!")
    print ("%d %d %d"%(USERID,TYPEID,ACTIONID))
    print ("saveing ...")

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

while True:
    x = input("菜单:")
    if warning == 1 and x != 'b':
        print ("正在录制中，需要暂停后在进行其他步骤！！")
        continue

    if x == 'a':
        start()
        continue
    if x == 'b':
        stop()
        continue
    if x == 'q':
        sys.exit
    else:
        continue

