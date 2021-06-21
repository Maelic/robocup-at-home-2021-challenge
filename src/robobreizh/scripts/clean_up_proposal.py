#!/usr/bin/env python

import Vision
import Navigation
import Movements

#State class
class State():
	NONE = 0
	SEARCH = 1
	MOVE = 2
	TAKE = 3
	DEPOSIT = 4
	END = 5

#Deposits information
class deposit_1():
	name = "deposit_1"
	coord = [0,0]
	handle_coord = [0,0,0]

class deposit_2():
	name = "deposit_2"
	coord = [0,0]
	handle_coord = [0,0,0]

class deposit_3():
	name = "deposit_3"
	coord = [0,0]
	handle_coord = [0,0,0]

class deposit_4():
	name = "deposit_4"
	coord = [0,0]
	handle_coord = [0,0,0]

class deposit_5():
	name = "deposit_5"
	coord = [0,0]
	handle_coord = [0,0,0]

#Object information
class object():
	name = "object"
	deposit_name = "default"
	coord = [0,0,0]

#Deposit coordinate
def associatedDeposit(detect_object):
	if detect_object.deposit_name == "deposit_1":
		deposit = deposit_1()
	elif detect_object.deposit_name == "deposit_2":
		deposit = deposit_2()
	elif detect_object.deposit_name == "deposit_3":
		deposit = deposit_3()
	elif detect_object.deposit_name == "deposit_4":
		deposit = deposit_4()
	elif detect_object.deposit_name == "deposit_5":
		deposit = deposit_5()

	return deposit

#State machine
def start():
	#Init
	nbTidyObject = 0
	previous_state = Stat.NONE
	state = State.SEARCH

	while True:

		if state == State.SEARCH:
			#Detect and localize not deposit object
			detected_object = Vision.searchObject()

			#Next state
			previous_state = state
			state = State.MOVE

		elif state == State.MOVE:
			#Go to the destination
			if previous_state == State.SEARCH:
				Navigation.moveTo(detect_object.coord)
			elif previous_state == State.TAKE:
				Navigation.moveTo(associatedDeposit(detect_object).coord)

			#Next state
			previous_state = state
			if previous_state == State.SEARCH:
				state = State.TAKE
			elif previous_state == State.TAKE:
				state = State.DEPOSIT

		elif state == State.TAKE:
			#Take not deposit object
			Movements.takeObject(detected_object)

			#Next state
			previous_state = state
			state = State.MOVE

		elif state == State.DEPOSIT:
			#Open deposit
			Movements.openDeposit(associatedDeposit(detect_object))

			#Deposit object
			Movements.depositObject(associatedDeposit(detect_object))

			#Close deposit
			Movements.closeDeposit(associatedDeposit(detect_object))

			#Number tidy object +1
			nbTidyObject = nbTidyObject + 1

			#Next state
			previous_state = state
			if nbTidyObject == 30:
				state = State.END
			else:
				state = State.SEARCH
		
		elif state == State.END:
			#Exit 
			break

if __name__ == "__main__":
	#Start state machine
	start()



#Vision:
#object() = Vision.searchObject()

#Navigation:
#Navigation.moveTo([x,y,z])

#Movements
#Movements.takeObject(object())
#Movements.openDeposit(deposit_x())
#Movements.depositObject(deposit_x())
#Movements.closeDeposit(deposit_x())