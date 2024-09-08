
#include "../app/statika.h"
#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace statika;

namespace UnitTest1
{
	TEST_CLASS(UnitTest1)
	{
	public:

		TEST_METHOD(TestReactionsForZeroForces)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 0.0;
			force.Fx_x = 0.0;
			force.Fy_x = 0.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			Assert::AreEqual(0.0, FAx, 1e-9, L"FAx is incorrect");
			Assert::AreEqual(0.0, FAy, 1e-9, L"FAy is incorrect");
			Assert::AreEqual(0.0, FBy, 1e-9, L"FBy is Incorrect");

		}

		TEST_METHOD(TestReactionsForHorizontalForce)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 5.0;
			force.Fy = 0.0;
			force.Fx_x = 5.0;
			force.Fy_x = 0.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			Assert::AreEqual(5.0, FAx, 1e-9, L"FAx is incorrect");
			Assert::AreEqual(0.0, FAy, 1e-9, L"FAy is incorrect");
			Assert::AreEqual(0.0, FBy, 1e-9, L"FBy is Incorrect");

		}

		TEST_METHOD(TestReactionsForVerticalForce)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 5.0;
			force.Fx_x = 0.0;
			force.Fy_x = 3.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {10.5, 3.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(3.5, FAy);
			Assert::AreEqual(1.5, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionsForUniformLoad)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 0.0;
			force.Fx_x = 0.0;
			force.Fy_x = 0.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 2.0;
			uniformLoad.x1 = 2.0;
			uniformLoad.x2 = 8.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {12.0, 2.0}, {21.0, 5.0}, {12.0, 8.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(6.0, FAy);
			Assert::AreEqual(6.0, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionsForMomentAtSupport)
		{

			Moment moment;
			moment.M = 10.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 0.0;
			force.Fx_x = 0.0;
			force.Fy_x = 0.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {-10.0, 0.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(1.0, FAy);
			Assert::AreEqual(-1.0, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionsForMoment)
		{

			Moment moment;
			moment.M = 10.0;
			moment.x = 4.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 0.0;
			force.Fx_x = 0.0;
			force.Fy_x = 0.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {4.0, 4.0},{-6.0, 4.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(1.0, FAy);
			Assert::AreEqual(-1.0, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionsForMomentAndForce)
		{

			Moment moment;
			moment.M = 10.0;
			moment.x = 7.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 5.0;
			force.Fx_x = 0.0;
			force.Fy_x = 3.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 0.0;
			uniformLoad.x1 = 0.0;
			uniformLoad.x2 = 0.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {13.5, 3.0},{11.5, 7.0}, {1.5, 7.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(4.5, FAy);
			Assert::AreEqual(0.5, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionForForceAndUniformLoad)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 5.0;
			force.Fx_x = 0.0;
			force.Fy_x = 2.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 2.0;
			uniformLoad.x1 = 4.0;
			uniformLoad.x2 = 8.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {14.4, 2.0}, {18.8, 4.0}, {11.6, 8.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(7.2, FAy);
			Assert::AreEqual(5.8, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionForFroceAndUniformLoad2)
		{

			Moment moment;
			moment.M = 0.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 5.0;
			force.Fx_x = 0.0;
			force.Fy_x = 7.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 2.0;
			uniformLoad.x1 = 2.0;
			uniformLoad.x2 = 8.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {15.0, 2.0}, {27.5, 7.0}, {19.0, 8.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(7.5, FAy);
			Assert::AreEqual(9.5, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}

		TEST_METHOD(TestReactionForForceMomentUniformLoad)
		{

			Moment moment;
			moment.M = 10.0;
			moment.x = 0.0;

			statika::Force force;
			force.Fx = 0.0;
			force.Fy = 5.0;
			force.Fx_x = 0.0;
			force.Fy_x = 6.0;

			statika::UniformLoad uniformLoad;
			uniformLoad.q = 2.0;
			uniformLoad.x1 = 2.0;
			uniformLoad.x2 = 8.0;

			statika::Beam beam;
			beam.L = 10.0;

			StaticEquilibrium proracun;

			double FAx = proracun.reactionOnSupportAx(force);
			double FAy = proracun.reactionOnSupportAy(beam, force, moment, uniformLoad);
			double FBy = proracun.reactionOnSupportBy(beam, force, moment, uniformLoad);

			proracun.calcInternalMoment(beam, moment, force, uniformLoad);

			const auto& internalMoments = proracun.getInternalMoments();

			std::vector<std::pair<double, double>> expectedInternalMoments = { {0.0, 0.0}, {-10.0, 0.0}, {8.0, 2.0}, {28.0, 6.0}, {16.0, 8.0}, {0.0, 10.0} };

			Assert::AreEqual(0.0, FAx);
			Assert::AreEqual(9.0, FAy);
			Assert::AreEqual(8.0, FBy);
			Assert::AreEqual(expectedInternalMoments.size(), internalMoments.size(), L"Size of internal moments doesn't match expected size.");
			for (size_t i = 0; i < internalMoments.size(); ++i) {
				Assert::AreEqual(expectedInternalMoments[i].first, internalMoments[i].first, 1e-9, L"Moment value mismatch");
				Assert::AreEqual(expectedInternalMoments[i].second, internalMoments[i].second, 1e-9, L"Position value mismatch");
			}
		}
	};
}
