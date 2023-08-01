#pragma once

//角速度带模型ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "AC_Math.hpp"
#include "Filters_LP.hpp"
#include "Filters_BS.hpp"
#include "complex.hpp"

//FT采样点数目(必须为偶数)
#define FT_N 80

class ESO_AngularRate_Base
{
	public:
		virtual ~ESO_AngularRate_Base() = default;
	
		virtual double get_T() const = 0;
		virtual double get_b() const = 0;
		virtual double get_u() const = 0;
	
		virtual void update_u( double u ) = 0;
		virtual double run( double v ) = 0;
	
		virtual double getVibeFreq1() const = 0;
		virtual double getVibeFreq2() const = 0;
		virtual const complex<double>* get_ftFreqs() const = 0;
	
		virtual double get_EsAngularRate() const = 0;
		virtual double get_EsDisturbance() const = 0;
		virtual double get_EsDisturbanceFilted() const = 0;
		virtual double get_EsAngularAcceleration() const = 0;
		virtual double get_EsMainPower() const = 0;
};

class ESO_AngularRate : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;
		double z2;
		
		double last_err;

		double Hz;
		double h;
	
		double vibeFreq1;
		double vibeFreq2;
		
		Filter_Butter_LP rate_filter;
		Filter_Butter2_BS rate_BSfilter1;
		Filter_Butter2_BS rate_BSfilter2;
	
		Filter_Butter_LP acc_filter;
		Filter_Butter2_BS acc_BSfilter1;
		Filter_Butter2_BS acc_BSfilter2;
	
		Filter_Butter_LP disturbance_filter;
	
		//傅里叶变换
		uint16_t ft_ind;
		double ft_in[FT_N];
		complex<double> ft_freqs[FT_N/2];
		//傅里叶变换参数
		static bool ft_coeffs_initialized;
		static complex<double> ft_coeffs[FT_N];
	
	public:	
		double beta1;
		double beta2;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
		
		double getVibeFreq1() const { return vibeFreq1; }
		double getVibeFreq2() const { return vibeFreq2; }
		const complex<double>* get_ftFreqs() const { return ft_freqs; }
		
		void init_ft();
	
		inline void init( uint8_t order, double T , double b , double beta1 , double beta2 ,
			uint8_t orderD, double betaD,
			double Hz )
		{
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			this->beta2 = beta2;
			rate_filter.set_cutoff_frequency( order, Hz, beta1 );
			rate_BSfilter1.set_inavailable();
			rate_BSfilter2.set_inavailable();
			acc_filter.set_cutoff_frequency( order, Hz, beta2 );
			acc_BSfilter1.set_inavailable();
			acc_BSfilter2.set_inavailable();
			disturbance_filter.set_cutoff_frequency( orderD, Hz, betaD );
			
			this->z1 = this->z2 = this->z_inertia = 0;
			
			this->T = T;	this->invT = 1.0 / T;
			this->b = b;
			
			vibeFreq1 = vibeFreq2 = 0;
			init_ft();
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
			this->z1 += this->h * ( this->z_inertia + this->z2 );
		}
		
		double run( double v );
		
		inline double get_EsAngularRate() const
		{
			return this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z2;
		}
		inline double get_EsDisturbanceFilted() const
		{
			return this->disturbance_filter.get_result();
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->z2 + this->z_inertia;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};

class ESO_AngularRateHeli : public ESO_AngularRate_Base
{
	private:
		double invT;
		double z_inertia;
		double z1;

		double last_v;
		double acc;
	
		double Hz;
		double h;
		
		double vibeFreq1;
		double vibeFreq2;
	
		Filter_Butter_LP rate_filter;
		Filter_Butter2_BS rate_BSfilter1;
		Filter_Butter2_BS rate_BSfilter2;
	
		Filter_Butter_LP disturbance_filter;
	
		//傅里叶变换
		uint16_t ft_ind;
		double ft_in[FT_N];
		complex<double> ft_freqs[FT_N/2];
		//傅里叶变换参数
		static bool ft_coeffs_initialized;
		static complex<double> ft_coeffs[FT_N];
	
	public:	
		double beta1;
	
		double T;	double get_T() const{ return this->T; };
		double b;	double get_b() const{ return this->b; };
		double u;	double get_u() const{ return this->u; };
	
		double getVibeFreq1() const { return vibeFreq1; }
		double getVibeFreq2() const { return vibeFreq2; }
		const complex<double>* get_ftFreqs() const { return ft_freqs; }
		
		void init_ft();
		
		inline void init( uint8_t order, double T , double b , double beta1 ,
			uint8_t orderD, double betaD,
			double Hz )
		{
			this->Hz = Hz;
			this->h = 1.0 / Hz;
			this->beta1 = beta1;
			rate_filter.set_cutoff_frequency( order, Hz, beta1 );
			rate_BSfilter1.set_inavailable();
			rate_BSfilter2.set_inavailable();
			disturbance_filter.set_cutoff_frequency( orderD, Hz, betaD );
			
			this->z1 = this->z_inertia = 0;
			
			this->T = T;	this->invT = 1.0f / T;
			this->b = b;
			
			vibeFreq1 = vibeFreq2 = 0;
			init_ft();
		}
		
		inline void update_u( double u )
		{
			this->u = u;
			this->z_inertia += this->h * this->invT * ( this->b*this->u - this->z_inertia );
		}
		
		double run( double v );
		
		inline double get_EsAngularRate() const
		{
			return this->z_inertia + this->z1;
		}
		inline double get_EsDisturbance() const
		{
			return this->z1;
		}
		inline double get_EsDisturbanceFilted() const
		{
			return this->disturbance_filter.get_result();
		}
		inline double get_EsAngularAcceleration() const
		{
			return this->acc;
		}
		inline double get_EsMainPower() const
		{
			return this->z_inertia;
		}
};