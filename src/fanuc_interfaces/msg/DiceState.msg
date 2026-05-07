# Current game state published after every significant event

int32 expected_pip      # Next pip we need to find (1-6)
int32 dj_retries        # Total retries accumulated by DJ
int32 bill_retries      # Total retries accumulated by BILL
string robot_holding    # 'DJ', 'BILL', or 'IN_TRANSIT'
