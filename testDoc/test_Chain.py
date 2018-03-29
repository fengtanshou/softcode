f = open("./log.txt", 'w+')
class Node():
    def __init__(self,value = None,next = None):
        self._value = value 
        self._next = next
        print ("self._vaule = ",self._value,file = f)
        print ("self._next = ",self._next , file =f)

class Chain():
    def __init__(self):
        self._head = None
        self.length = 0
    
    def isEmpty(self):
        return self.length == 0

    def append(self,value):
        if isinstance(value,Node):
            print ("value00:%s"%value)
            node = value
        else:
            print ("value11:%s"%value)
            node = Node(value)

        if self._head == None:
            print ("000000000")
            self._head = node
        else:
            print ("111111111")
            be_node = self._head
            while be_node._next:
                print ("hello ")
                be_node = be_node._next
            print ("world ")
            be_node._next = node
        self.length += 1

    def delete(self, index):
        print("index = ",index)
        if type(index) is int:
            if ( index > self.length ) or ( index <= 0 ):
            # 索引值超出范围直接提示并且退出
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

chain = Chain()
chain.append('A')
chain.append('B')
chain.append('C')
chain.append('D')
chain.printlist()
print ("--------------------------")
chain.delete(1)
chain.printlist()

f.close()
