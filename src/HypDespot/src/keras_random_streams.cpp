#include <despot/keras_random_streams.h>
#include <despot/util/seeds.h>
#include <despot/core/globals.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <assert.h>
#include <random>
using namespace std;

namespace despot {


KerasRandomStreams::KerasRandomStreams(int num_streams, int length, int latent_dimension) :
	RandomStreams(num_streams, length),
	latent_dimension_(latent_dimension){

	if(latent_dimension > 0)
	{
		 //unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		vector<unsigned> seeds = Seeds::Next(num_streams);
		vector<Random> uniform_random_gen;
		for(int i = 0; i < num_streams; i++)
		{
			Random random(seeds[i]);
			uniform_random_gen.push_back(random);
		}

		 std::default_random_engine generator( Globals::config.root_seed);
		 std::normal_distribution<float> distribution(0,1);
		//length = depth
		//num_strems = num_particles
		keras_streams_.resize(length);
		keras_uniform_streams_.resize(length);
		for(int i = 0; i < length; i++)
		{
			keras_streams_[i].resize(num_streams*latent_dimension);
			keras_uniform_streams_[i].resize(num_streams);
			for(int j = 0; j < (num_streams*latent_dimension); j++)
			{
				keras_streams_[i][j] = distribution(generator);
				if(j % latent_dimension == 0)
				{// Push new random number only for first value
					keras_uniform_streams_[i][j/latent_dimension] = (float)uniform_random_gen[j/latent_dimension].NextDouble();
				}

			}
		}
	}


}


const std::vector<float>& KerasRandomStreams::KerasALLParticlesEntry(int position) const {
	if(position == -1)
	{
		return keras_streams_[position_];
	}
	else
	{
		return keras_streams_[position];
	}
}

const void KerasRandomStreams::KerasParticlesEntry(const std::vector<State*>& particles, std::vector<float>& random_vector, bool uniform_stream, int position) const
{
	int p = position;
	if(position == -1)
		{
			p = position_;
		}
	int num_begin = 0;
	int num_end = 0;
	for(int i = 0; i < particles.size(); i++)
	{
		int particle_id = particles[i]->scenario_id;
		if(particle_id == (num_end))
		{
			num_end++;
		}
		else
		{
			if(uniform_stream)
			{
				random_vector.insert(random_vector.end(),keras_uniform_streams_[p].data() + num_begin, keras_uniform_streams_[p].data() + (num_end));

			}
			else
			{
				random_vector.insert(random_vector.end(),keras_streams_[p].data() + num_begin*latent_dimension_, keras_streams_[p].data() + (num_end)*latent_dimension_);
			}
			num_begin = particle_id;
			num_end = num_begin +1;
		}
	}
	if(num_end > num_begin)
	{
		if(uniform_stream)
		{
			random_vector.insert(random_vector.end(),keras_uniform_streams_[p].data() + num_begin, keras_uniform_streams_[p].data() + (num_end));

		}
		else
		{
			random_vector.insert(random_vector.end(),keras_streams_[p].data() + num_begin*latent_dimension_, keras_streams_[p].data() + (num_end)*latent_dimension_);
		}
	}

}


const void KerasRandomStreams::KerasParticlesEntry(const std::vector<int>& particles, std::vector<float>& random_vector,bool uniform_stream, int position) const
{
	int p = position;
	if(position == -1)
		{
			p = position_;
		}
	int num_begin = 0;
	int num_end = 0;
	for(int i = 0; i < particles.size(); i++)
	{
		int particle_id = particles[i];
		if(particle_id == (num_end))
		{
			num_end++;
		}
		else
		{
			if(uniform_stream)
			{
				random_vector.insert(random_vector.end(),keras_uniform_streams_[p].data() + num_begin, keras_uniform_streams_[p].data() + (num_end));
			}
			else
			{
				random_vector.insert(random_vector.end(),keras_streams_[p].data() + num_begin*latent_dimension_, keras_streams_[p].data() + (num_end)*latent_dimension_);
			}
			num_begin = particle_id;
			num_end = num_begin +1;
		}
	}
	if(num_end > num_begin)
	{
		if(uniform_stream)
		{
			random_vector.insert(random_vector.end(),keras_uniform_streams_[p].data() + num_begin, keras_uniform_streams_[p].data() + (num_end));
		}
		else
		{
			random_vector.insert(random_vector.end(),keras_streams_[p].data() + num_begin*latent_dimension_, keras_streams_[p].data() + (num_end)*latent_dimension_);
		}
	}

}
} // namespace despot