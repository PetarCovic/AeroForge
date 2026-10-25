#include <telemetry/CsvTelemetryWriter.h>

#include <stdexcept>
#include <charconv>
#include <array>
#include <string>
#include <system_error>

namespace aeroforge::telemetry
{
	namespace
	{
		void appendDouble(std::string& buffer, double value)
		{
			std::array<char, 64> temp;
			auto result=std::to_chars(temp.data(), temp.data() + temp.size(), value, std::chars_format::general, 17);

			if (result.ec != std::errc{})
			{
				throw std::runtime_error("double conversion failed");
			}

			buffer.append(temp.data(), static_cast<std::size_t>(result.ptr - temp.data()));
		}
	}

	void CsvTelemetryWriter::write(std::ostream& output, const std::vector<TelemetrySample>& samples)
	{
		if (!output)
		{
			throw std::runtime_error("Output stream is not writable");
		}

		output	<< "time_s,"
				<< "position_n_m,position_e_m,position_d_m,"
				<< "velocity_n_mps,velocity_e_mps,velocity_d_mps,"
				<< "attitude_q_w,attitude_q_x,attitude_q_y,attitude_q_z,"
				<< "omega_x_radps,omega_y_radps,omega_z_radps,"
				<< "estimated_position_n_m,estimated_position_e_m,estimated_position_d_m,"
				<< "estimated_velocity_n_mps,estimated_velocity_e_mps,estimated_velocity_d_mps,"
				<< "estimated_attitude_q_w,estimated_attitude_q_x,estimated_attitude_q_y,estimated_attitude_q_z,"
				<< "estimated_omega_x_radps,estimated_omega_y_radps,estimated_omega_z_radps,"
				<< "attitude_error_x_rad,attitude_error_y_rad,attitude_error_z_rad,"
				<< "commanded_moment_x_nm,commanded_moment_y_nm,commanded_moment_z_nm,"
				<< "actual_moment_x_nm,actual_moment_y_nm,actual_moment_z_nm,"
				<< "throttle,gimbal_pitch_rad,gimbal_yaw_rad,controller_saturated\n";
		
		std::string row;
		row.reserve(1024);

		for (const TelemetrySample& sample : samples)
		{
			row.clear();

			appendDouble(row, sample.timeS);
			row.push_back(',');
			appendDouble(row, sample.positionN.x);
			row.push_back(',');
			appendDouble(row, sample.positionN.y);
			row.push_back(',');
			appendDouble(row, sample.positionN.z);
			row.push_back(',');

			appendDouble(row, sample.velocityN.x);
			row.push_back(',');
			appendDouble(row, sample.velocityN.y);
			row.push_back(',');
			appendDouble(row, sample.velocityN.z);
			row.push_back(',');

			appendDouble(row, sample.attitudeNB.w);
			row.push_back(',');
			appendDouble(row, sample.attitudeNB.x);
			row.push_back(',');
			appendDouble(row, sample.attitudeNB.y);
			row.push_back(',');
			appendDouble(row, sample.attitudeNB.z);
			row.push_back(',');

			appendDouble(row, sample.angularVelocityB.x);
			row.push_back(',');
			appendDouble(row, sample.angularVelocityB.y);
			row.push_back(',');
			appendDouble(row, sample.angularVelocityB.z);
			row.push_back(',');

			appendDouble(row, sample.estimatedPositionN.x);
			row.push_back(',');
			appendDouble(row, sample.estimatedPositionN.y);
			row.push_back(',');
			appendDouble(row, sample.estimatedPositionN.z);
			row.push_back(',');

			appendDouble(row, sample.estimatedVelocityN.x);
			row.push_back(',');
			appendDouble(row, sample.estimatedVelocityN.y);
			row.push_back(',');
			appendDouble(row, sample.estimatedVelocityN.z);
			row.push_back(',');

			appendDouble(row, sample.estimatedAttitudeNB.w);
			row.push_back(',');
			appendDouble(row, sample.estimatedAttitudeNB.x);
			row.push_back(',');
			appendDouble(row, sample.estimatedAttitudeNB.y);
			row.push_back(',');
			appendDouble(row, sample.estimatedAttitudeNB.z);
			row.push_back(',');

			appendDouble(row, sample.estimatedAngularVelocityB.x);
			row.push_back(',');
			appendDouble(row, sample.estimatedAngularVelocityB.y);
			row.push_back(',');
			appendDouble(row, sample.estimatedAngularVelocityB.z);
			row.push_back(',');

			appendDouble(row, sample.attitudeErrorB.x);
			row.push_back(',');
			appendDouble(row, sample.attitudeErrorB.y);
			row.push_back(',');
			appendDouble(row, sample.attitudeErrorB.z);
			row.push_back(',');

			appendDouble(row, sample.commandedMomentB.x);
			row.push_back(',');
			appendDouble(row, sample.commandedMomentB.y);
			row.push_back(',');
			appendDouble(row, sample.commandedMomentB.z);
			row.push_back(',');

			appendDouble(row, sample.actualMomentB.x);
			row.push_back(',');
			appendDouble(row, sample.actualMomentB.y);
			row.push_back(',');
			appendDouble(row, sample.actualMomentB.z);
			row.push_back(',');

			appendDouble(row, sample.throttle);
			row.push_back(',');
			appendDouble(row, sample.gimbalPitchRad);
			row.push_back(',');
			appendDouble(row, sample.gimbalYawRad);
			row.push_back(',');
			row.push_back((sample.controllerSaturated ? '1' : '0'));
			row.push_back('\n');

			output.write(row.data(), static_cast<std::streamsize>(row.size()));
		}

		if (!output)
		{
			throw std::runtime_error("Output stream is not writable");
		}
	}
}