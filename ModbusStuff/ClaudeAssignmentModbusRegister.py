### MODBUS REGISTER MAPS
COIL_REGISTER_MAP = {
# DJ - Robot A
"DJ_Has_Dice": 0, # Bill has let go, DJ is holding die
# Bill - Robot B
"Bill_Has_Dice": 1, # Bill has set position registers and is ready to pass
# Shared
"Ready_For_Pickup":2, # Dice is ready to be picked up from conveyor
"Cycle_Active": 3, # Clycle Active bit
}
HOLDING_REGISTER_MAP = {
"Total_Pip_Count": 0, # Total pip count
"Total_Retries": 1, # Total number of retries (dice flips)
"Bill_Retries": 2, # Number of retries for Bill
"DJ_Retries": 3, # Number of retries for DJ
"Last_Known_Pip": 4, # Last known pip count (current iteration)
}