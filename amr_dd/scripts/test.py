active_ = 0
state_ = 2
msg = 0
while 1:
    print(msg)
    if not active_:
        continue
    msg = 1
    if state_ == 0:
        msg = 2
    elif state_ == 1:
        msg = 3
    print(msg)