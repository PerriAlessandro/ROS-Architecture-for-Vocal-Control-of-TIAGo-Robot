
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
    return mode, kind

def unfold(word):
    print('unfold')
    mode = 1
    kind = 2
    return mode, kind

def maximum(word):
    print('maximum')
    mode = 1
    kind = 3
    return mode, kind

def floor(word):
    print('floor')
    mode = 1
    kind = 4
    return mode, kind

def shake(word):
    print('shake')
    mode = 1
    kind = 5
    return mode, kind

def offer(word):
    print('offer')
    mode = 1
    kind = 6
    return mode, kind

def surroundings(word):
    print('surroundings')
    mode = 1
    kind = 7
    return mode, kind

def tour(word):
    print('tour')
    mode = 1
    kind = 8
    return mode, kind

def close(word):
    print('close')
    mode = 1
    kind = 9
    return mode, kind

def half(word):
    print('half')
    mode = 1
    kind = 10
    return mode, kind

def gym(word):
    print('gym')
    mode = 1
    kind = 11
    return mode, kind

def home(word):
    print('home')
    mode = 1
    kind = 12
    return mode, kind

#### Mode 2 ####
def go(word):
    print('OKKK, Lets gooo')
    mode = 2
    kind = 1
    return mode, kind

def accelerate(word):
    print('accelerate')
    mode = 2
    kind = 6
    return mode, kind

def decelerate(word):
    print('decelerate')
    mode = 2
    kind = 7
    return mode, kind

def reset(word):
    print('reset')
    mode = 2
    kind = 8
    return mode, kind

def left(word):
    print('left')
    mode = 2
    if 'straight' in word:
            kind = 5
    else:
            kind = 2
    return mode, kind

def right(word):
    print('right')
    mode = 2
    if 'straight' in word:
            kind = 4
    else:
            kind = 3
    return mode, kind


def backward(word):
    print('BACKWARDS')
    mode = 2
    kind = 9
    return mode, kind

def stop(word):
    print('STOP')
    mode = 2
    kind = -1
    return mode, kind

#### Mode 3 ####
def arm(word):
    print('Arm: '+word)

    mode=3
    if 'up' in word:
            kind = 1
    elif 'down' in word:
            kind = 2
    else:
        kind=-1    
    return mode, kind

def elbow(word):

    mode=3
    if 'up' in word:
            kind = 3
    elif 'down' in word:
            kind = 4
    else:
        kind=-1    
    return mode, kind


#### Default ####
def default(word):
    print ("Command not recognized, try again..")
    mode = -1
    kind = -1
    return mode, kind

# Dictionary that contain all the possible vocal commands

switcher = {

    'hello': hello,

    'unfold': unfold,

    'maximum': maximum,

    'floor': floor,

    'shake': shake,

    'offer': offer,

    'surroundings': surroundings,

    'tour': tour,

    'close': close,

    'half': half,

    'gym': gym,

    'home': home,

    'go': go,

    'left' : left,

    'right': right,

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

