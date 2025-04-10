time ts
uint16 signal_lvl # Not the same as the one display one the router webpage for w/e reasons
string reg_status
string operator
string network
string cell_id
string phone_num
uint8 active_sim
int16 rssi # Represents the entire received power including the wanted power from the serving cell as well as all cochannel power.
int16 rsrp # The average power received from a single reference signal, and its typical range is around -44dbm (good) to -140dbm(bad).
int16 rsrq # Indicates quality of the received signal, and its range is typically -19.5dB(bad) to -3dB (good).
int16 sinr