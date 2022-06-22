
import matplotlib.pyplot as plt

def get_output_data():
    
    fname ='result.txt'
    with open(fname) as f:
        data = f.read().splitlines()
   
    ls=[]
    num=0
    for each in data[:]:
        num+=1
        tmp_list=[]
        m = []
        n = []
        if(num%2)==0:
            
            each=each[1:-1] 

            d=each.split('),(')
            for tmp in d:         
                
                e=tmp.split(',')              
            
                m.append(e[1])
                n.append(e[2])
            tmp_list.append(m)
            tmp_list.append(n)
            
            ls.append(tmp_list)

        
       
    return ls
if __name__ == '__main__':

    ls=get_output_data()

    i=0
    for e in ls:
       
        i+=1
        plt.plot(e[0],e[1],"g")
        # plt.plot(e[1],e[0])
        plt.grid()
        plt.title('path{id}'.format(id=i))
        plt.savefig('{id}.png'.format(id=i))
        plt.close()

    