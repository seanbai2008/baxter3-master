#!/usr/bin/env python

import argparse
def adjust(x,y,z,roll,pitch,yaw):
    state = False
    def set_vars(input_var,x,y,z,roll,pitch,yaw):
        if input_var[0]=='x':
		x+=float(raw_input("Enter amount x should be adjusted by: "))
		print 'x adjusted to be %d' % x
	elif input_var[0]=='y':
		y+=float(raw_input("Enter amount y should be adjusted by: "))
		print 'y adjusted to be %d' % y
	elif input_var[0]=='z':
		z+=float(raw_input("Enter amount z should be adjusted by: "))
		print 'z adjusted to be %d' % z
	elif input_var[0]=='r':
		roll+=float(raw_input("Enter amount roll should be adjusted by: "))
		print 'roll adjusted to be %d' % roll
	elif input_var[0]=='p':
		pitch+=float(raw_input("Enter amount pitch should be adjusted by: "))
		print 'pitch adjusted to be %d' % pitch
	elif input_var[0]=='a':
		yaw+=float(raw_input("Enter amount yaw should be adjusted by: "))
		print 'yaw adjusted to be %d' % yaw
	else:
		print 'Something is wrong'
	return (x,y,z,roll,pitch,yaw)

    def complete(x,y,z,roll,pitch,yaw):
	state=True
	return (x,y,z,roll,pitch,yaw)

    while(state == False):
    	adjust_yn = raw_input("Is this position ok? (y/n): ")
    	if adjust_yn[0]=='y':
		state = True
    		return (x,y,z,roll,pitch,yaw)
    	elif adjust_yn[0]=='n':
    		bindings = {
    			#key: (function, args, description)
        		'a': (set_vars, ['x'], "Adjust x Position"),
        		's': (set_vars, ['y'], "Adjust y Position"),
        		'd': (set_vars,['z'],"Adjust z position"),
        		'j': (set_vars,['r'],"Adjust Roll"),
        		'k': (set_vars,['p'],"Adjust Pitch"),
                        'l': (set_vars,['a'],"Adjust Yaw"),
			'q': (complete,[], "Quit"),
     		}  
		print("key bindings: ")
		for key, val in sorted(bindings.items(),
        	    key=lambda x: x[1][2]):
        	    print("  %s: %s" % (key, val[2]))

		command_choice=raw_input("Which variable would you like to adjust? : ")
            	if command_choice in bindings:
                	(x,y,z,roll,pitch,yaw)=bindings[command_choice][0](bindings[command_choice][1],x,y,z,roll,pitch,yaw)
            	else:
                	print("key bindings: ")
                	for key, val in sorted(bindings.items(),
                        	key=lambda x: x[1][2]):
                    		print("  %s: %s" % (key, val[2]))   
    	else:
	        print 'Invalid input'	
	 

def main():
	(x,y,z,roll,pitch,yaw)=adjust(1,1,1,1,1,1)
	print 'x is now %d, y is now %d, z is now %d' % (x,y,z)
	print 'roll is now %d, pitch is now %d, yaw is now %d' %(roll,pitch,yaw)

if __name__ == '__main__':
    main()
