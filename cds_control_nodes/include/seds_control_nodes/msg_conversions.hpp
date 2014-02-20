#include <dlr_msgs/rcu2tcu.h>
#include <realtime_bridge_msgs/ImpedanceCommand.h>

void fromMsg(const dlr_msgs::rcu2tcu_Robot& msg, KDL::JntArray& q)
{
  assert(msg.q.size() == q.rows());

  for(unsigned int i=0; q.rows(); i++)
    q(i) = msg.q[i];
}

void fromMsg(const dlr_msgs::rcu2tcu& msg, KDL::JntArray& q)
{
  fromMsg(msg.robot, q);
}

void toMsg(const KDL::JntArray& qdot, realtime_bridge_msgs::ImpedanceCommand& msg)
{
  assert(qdot.rows() == msg.velocity.size());

  for(unsigned int i=0; i<qdot.rows(); i++)
    msg.velocity[i] = qdot(i);
}
