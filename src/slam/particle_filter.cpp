#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <random>

using namespace std;


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    std::default_random_engine generator;

    std::normal_distribution<double> distributionX(0.0, 0.5);
    std::normal_distribution<double> distributionY(0.0, 0.5);
    std::normal_distribution<double> distributionT(0.0, 3.1415926/10.0);

    for(int i = 0; i < kNumParticles_; i++){
        posterior_.at(i).pose = pose;
        posterior_.at(i).parent_pose = pose;
        posterior_.at(i).weight = 1;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }

    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior;

    prior = posterior_;

    double M = double(prior.size());
    double maxR = 1.0 / M;

    double weightSum = 0;

    for(int i = 0; i < int(prior.size()); i++){
        weightSum += prior.at(i).weight;
    }

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,maxR);

    double U = distribution(generator) * weightSum;
    double c = prior.at(0).weight;

    int i = 0;

    for(int m = 0; m < int(prior.size()); m++){
        if (m > 1){ U += ((m - 1) * maxR) * weightSum; }

        while (U > c){
          i += 1;
          c += prior.at(i).weight;
        }
        prior.at(m) = prior.at(i);
    }

    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    proposal = prior;

    for(int i = 0; i < int(proposal.size()); i++){
        proposal.at(i) = actionModel_.applyAction(prior.at(i));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;

    posterior = proposal;

    for(int i = 0; i < int(posterior.size()); i++){
        posterior.at(i).weight = sensorModel_.likelihood(proposal.at(i), laser, map);
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    double maxWeight = posterior.at(0).weight;

    for(int i = 1; i < int(posterior.size()); i++){
        if(posterior.at(i).weight > maxWeight){
          maxWeight = posterior.at(i).weight;
          pose = posterior.at(i).pose;
        }
    }

    return pose;
}
