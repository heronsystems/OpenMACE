#pragma once

#ifdef MACE_USE_MESSAGE_INFO
#define MACE_HAVE_GET_MESSAGE_INFO
/*
  return the message_info struct for a message
*/
MACE_HELPER const mace_message_info_t *mace_get_message_info(const mace_message_t *msg)
{
	static const mace_message_info_t mace_message_info[] = MACE_MESSAGE_INFO;
        /*
	  use a bisection search to find the right entry. A perfect hash may be better
	  Note that this assumes the table is sorted with primary key msgid
	*/
	uint32_t msgid = msg->msgid;
        uint32_t low=0, high=sizeof(mace_message_info)/sizeof(mace_message_info[0]);
        while (low < high) {
            uint32_t mid = (low+1+high)/2;
            if (msgid < mace_message_info[mid].msgid) {
                high = mid-1;
                continue;
            }
            if (msgid > mace_message_info[mid].msgid) {
                low = mid;
                continue;
            }
            low = mid;
            break;
        }
        if (mace_message_info[low].msgid == msgid) {
            return &mace_message_info[low];
        }
        return NULL;
}
#endif // MACE_USE_MESSAGE_INFO


