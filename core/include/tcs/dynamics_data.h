#ifndef _DYNAMICS_DATA_H_
#define _DYNAMICS_DATA_H_

#include <tcs/time.h>
#include <tcs/message.h>

//-----------------------------------------------------------------------------

struct dynamics_data_t {
public:
  realtime_t   t;
  realtime_t   dt;

  state_t      prey_state;
  control_t    prey_control;

  state_t      pred_state;
  control_t    pred_control;

  double       prey_ke;
  double       pred_ke;
};

//-----------------------------------------------------------------------------

#endif // _DYNAMICS_DATA_H_
