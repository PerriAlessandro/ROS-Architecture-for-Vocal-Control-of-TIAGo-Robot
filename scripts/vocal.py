
###### Vocal Commands ########

'''

A special function is associated with each voice command that sets 
two integers (mode, type) recognizable by the other nodes.

'''

#### Mode 1 ####
def hello(word):
    print('Hello from TIAGo')
    mode = 1
    kind = 1
    name = 'hello'
    info = 'say hello with the arm'
    return mode, kind,name,info

def unfold(word):
    print('unfold')
    mode=1
    if 'left' in word:
            kind = 2
            name = 'open left'
            info = 'opening left arm'
    elif 'right' in word:
            kind = 3
            name = 'open right'
            info = 'opening right arm'
    elif 'both' in word:
            kind = 4
            name = 'open both'
            info = 'opening both arms'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

def home(word):
    print('home')
    mode=1
    if 'left' in word:
            kind = 5
            name = 'home left'
            info = 'home left arm'
    elif 'right' in word:
            kind = 6
            name = 'home right'
            info = 'home right arm'
    elif 'both' in word:
            kind = 7
            name = 'home both'
            info = 'home both arms'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

def close(word):
    print('close')
    mode = 1
    if 'left' in word:
            kind = 8
            name = 'close left'
            info = 'close left arm'
    elif 'right' in word:
            kind = 9
            name = 'close right'
            info = 'close right arm'
    elif 'both' in word:
            kind = 10
            name = 'close both'
            info = 'close both arms'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

def maximum(word):
    print('maximum')
    mode = 1
    if 'left' in word:
            kind = 11
            name = 'maximum left'
            info = 'maximum left arm'
    elif 'right' in word:
            kind = 12
            name = 'maximum right'
            info = 'maximum right arm'
    elif 'both' in word:
            kind = 13
            name = 'maximum both'
            info = 'maximum both arms'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

def reach(word):
    print('reach')
    mode = 1
    if 'vertical' in word:
            kind = 14
            name = 'vertical reach'
            info = 'reaching vertical position'
    elif 'horizontal' in word:
            kind = 15
            name = 'horizontal reach'
            info = 'reaching horizontal position'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

def offer(word):
    print('offer')
    mode = 1
    if 'left' in word:
            kind = 16
            name = 'offer left'
            info = 'offer left arm'
    elif 'right' in word:
            kind = 17
            name = 'offer right'
            info = 'offer right arm'
    elif 'both' in word:
            kind = 18
            name = 'offer both'
            info = 'offer both arms'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info

#### Mode 2 ####
def go(word):
    print('OKKK, Lets gooo')
    mode = 2
    kind = 1
    name = 'go'
    info = 'goes straight-forward'
    return mode, kind,name,info

def accelerate(word):
    print('accelerate')
    mode = 2
    kind = 6
    name = 'accelerate'
    info = 'accelerates'
    return mode, kind,name,info

def decelerate(word):
    print('decelerate')
    mode = 2
    kind = 7
    name = 'decelerate'
    info = 'decelerates'
    return mode, kind,name,info

def reset(word):
    print('reset')
    mode = 2
    kind = 8
    name = 'reset'
    info = 'resets the velocity'
    return mode, kind,name,info

def turn(word):
    if 'left' in word:
        mode = 2
        if 'straight' in word:
            kind = 5
            name = 'straight left'
            info = 'goes straight-left'
        else:
            kind = 2
            name = 'left'
            info = 'turns left'
    elif 'right' in word:
        mode = 2
        if 'straight' in word:
            kind = 4
            name = 'straight right'
            info = 'goes straight-right'
        else:
            kind = 3
            name = 'right'
            info = 'turns right'

    return mode, kind,name,info

def backward(word):
    print('BACKWARDS')
    mode = 2
    kind = 9
    name = 'backward'
    info = 'goes backwards'
    return mode, kind,name,info

def stop(word):
    print('STOP')
    mode = 2
    kind = -1
    name = 'stop'
    info = 'stops the motion'
    return mode, kind,name,info

#### Mode 3 ####
def arm(word):
    print('Arm: '+word)

    mode=3
    if 'up' in word:
        if 'right' in word:
            kind = 5
            name = 'arm right up'
            info = 'increases the shoulder joint of right arm'
        else:
            kind = 1
            name = 'arm left up'
            info = 'increases the shoulder joint of left arm'            
    elif 'down' in word:
        if 'right' in word: 
            kind = 6
            name = 'arm right down'
            info = 'decreases the shoulder joint of right arm'
        else:
            kind = 2
            name = 'arm left down'
            info = 'decreases the shoulder joint of left arm'            
    else:
        kind=-1
        name = ''
        info = ''    
    return mode, kind,name,info

def elbow(word):

    mode=3
    if 'up' in word:
        if 'right' in word:
            kind = 7
            name = 'elbow right up'
            info = 'increases the elbow joint of right arm'
        else:
            kind = 3
            name = 'elbow left up'
            info = 'increases the elbow joint of left arm'
    elif 'down' in word:
        if 'right' in word:
            kind = 8
            name = 'elbow right up'
            info = 'decreases the elbow joint of right arm'
        else:
            kind = 4
            name = 'elbow left down'
            info = 'decreases the elbow joint of left arm'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info


#### Default ####
def default(word):
    print ("Command not recognized, try again..")
    mode = -1
    kind = -1
    name = ''
    info = ''
    return mode, kind,name,info

# Dictionary that contain all the possible vocal commands

switcher = {

    'hello': hello,

    'unfold': unfold,

    'maximum': maximum,

    'reach': reach,

    'offer': offer,

    'close': close,

    'home': home,

    'go': go,

    'turn' : turn,

    'backward': backward,

    'accelerate': accelerate,

    'decelerate': decelerate,

    'reset': reset,

    'arm': arm,

    'elbow': elbow,

    'stop': stop
}


def word(word):

    """

    Function to call the proper action according to the vocal input

    """
    command=""
    # Browsing in the dictionary to know if the vocal input is right 
    # or not 

    for key in switcher:
        if key in word:
            command=key

    # getting the riht function to call from the dictionary 
    function = switcher.get(command,default)
    return function(word)

