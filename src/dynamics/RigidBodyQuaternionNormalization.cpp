#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <math/Quaternion.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	void normalizeRigidBodyQuaternion(simulation::State& state)
	{
		if (state.size() != 13)
		{
			throw std::invalid_argument("State must have 13 elements");
		}

		math::Quaternion quaternion(state[6], state[7], state[8], state[9]);

		if (!quaternion.isFinite())
		{
			throw std::invalid_argument("quaternion elements of state must be finite");
		}

		double quaternionNorm=quaternion.norm();

		if (quaternionNorm == 0)
		{
			throw std::invalid_argument("quaternionNorm must be nonzero");
		}

		quaternion[0] /= quaternionNorm;
		quaternion[1] /= quaternionNorm;
		quaternion[2] /= quaternionNorm;
		quaternion[3] /= quaternionNorm;

		if (!quaternion.isFinite())
		{
			throw std::overflow_error("quaternion must be finite");
		}

		state[6] = quaternion[0];
		state[7] = quaternion[1];
		state[8] = quaternion[2];
		state[9] = quaternion[3];
	}
}