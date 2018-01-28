# -*- coding: utf-8 -*-
"""
Created on Sat Dec 16 15:14:11 2017

@author: Jak

Sequence detector/testing
    given a noisy, un-debounded input, detect a sequence

"""



def detectSequence(buttonsUp, priorState, sequence):
    #print buttonsUp, sequence[priorState], buttonsUp[sequence[priorState]]
    if buttonsUp[sequence[priorState]]:
        nextState = priorState + 1
        if nextState == len(sequence):
            return 0, True
    else:
        nextState = priorState
        
    return nextState, False
        
def dummyButtonSequence():
    b = []
    b.append([False, True, False, False, False])
    b.append([False, False, False, False, True])
    b.append([False, False, True, False, False])
    b.append([False, True, False, False, False])
    b.append([False, False, False, True, False])
    b.append([False, False, False, False, False])
    b.append([False, False, False, True, False])
    b.append([False, False, False, False, False])

    return b
        
def go():
    numSequences = 2
    sequences = [[1, 4, 2, 1, 3, 3], \
                 [1, 4, 0]]
    
    sequenceState = [0]*numSequences
    sequenceTimeout = [10]*numSequences
    
    lastb = dummyButtonSequence()[0]
    for si in xrange(numSequences):
        for b in dummyButtonSequence()[1:]:
            buttonUp = [True if (lastb[i] and not b[i]) else False for i in xrange(len(b))]            
            print b, buttonUp
            lastb = b
            s = sequences[si]
            priorState = sequenceState[si]
            nextState, matched = detectSequence(buttonUp, priorState, s)
            #print nextState, matched
            if matched:
                print "sequence", si, True
            if priorState == nextState:
                sequenceTimeout[si] -= 1
                if sequenceTimeout[si] == 0:
                    sequenceTimeout[si] = 10
                    nextState = 0
            sequenceState[si] = nextState
                
    


if __name__ == "__main__":
    go()


