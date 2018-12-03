/*
Copyright (c) 2018 Todd Stellanova
LICENSE: See LICENSE file
*/

extern crate sensulator;
use sensulator::{MeasureVal, Sensulator};

extern crate rand;
use rand::distributions::{Normal, Distribution};

extern crate statistical;

const REL_ERR : MeasureVal  = 12 as MeasureVal;
const ABS_ERR : MeasureVal = 100 as MeasureVal;
const DEFAULT_AIR_PRESS: MeasureVal = 101325 as MeasureVal; 
const VEL_MULT: MeasureVal = (2.0f32/ 1.225f32) as MeasureVal;

const SPEED_STEPS: u32 = 100;
/// Max speed in m/s
const MAX_SPEED: MeasureVal = 45 as MeasureVal; 

struct DPressPair {
    actual_pressure: MeasureVal,
    dpress: MeasureVal,
}

fn init_sensulator(abs_err: MeasureVal, rel_err: MeasureVal, ctr_val: MeasureVal) -> Sensulator{
  let mut senso = Sensulator::new();
  senso.set_absolute_error_range(abs_err);
  senso.set_relative_error(rel_err);
  senso.set_center_value(ctr_val);
  senso
}



fn sdp33_test() {
  let speed_scale: MeasureVal =  MAX_SPEED / (SPEED_STEPS as MeasureVal);
  let density_fac: MeasureVal = (1.225 / 2.0);

  
  for step in 1 .. SPEED_STEPS { 
    let speed = (step as MeasureVal) * speed_scale;
    // dP = v^2 * (1.225/2) = dP
    let dp:MeasureVal = (speed*speed)*density_fac;
    let mut abs_err = 0.06f32 * dp;
    let mut rel_err = 0.03f32 * dp;
    if (dp < 500f32) {
      abs_err = 0.03f32 * dp;
      rel_err = 0.005f32 * dp;
    }
    
    let mut dp_sensor = init_sensulator(abs_err, rel_err, dp);
    let sampled_dp =  dp_sensor.read();
    
    let rel_err = 100f32*((sampled_dp - dp)/dp).abs();
    println!("speed: {:.3} \tdp: {:.3} \tsampled_dp: {:.3} \trel_err: {:.3}",speed, dp, sampled_dp, rel_err);
  }
}

fn ever_increasing_velocity() {
  let speed_scale: MeasureVal =  MAX_SPEED / (SPEED_STEPS as MeasureVal);
  let density_fac: MeasureVal = (1.225 / 2.0);
  let mut sensor_a = init_sensulator(ABS_ERR, REL_ERR, DEFAULT_AIR_PRESS);
  let mut sensor_b = init_sensulator(ABS_ERR, REL_ERR, DEFAULT_AIR_PRESS);
  
  for step in 1 .. SPEED_STEPS { 
    let speed = (step as MeasureVal) * speed_scale;
    // v^2 * (1.225/2) = dP
    let dp:MeasureVal = (speed*speed)*density_fac;
    let setpoint = DEFAULT_AIR_PRESS + dp;
    sensor_b.set_center_value(setpoint);
    let sampled_dp =  sensor_b.read() - sensor_a.read();
    let rel_err = 100f32*((sampled_dp - dp)/dp).abs();
    let mut velocity = VEL_MULT * sampled_dp.abs();
    velocity = velocity.sqrt();
    
    println!("speed: {:.3} \tsdp: {:.3} \trel_err: {:.3}",speed, sampled_dp, rel_err);
    // println!("speed: {} \tdp: {} \tsampled_dp: {} \trel_err: {}", speed, dp, sampled_dp, rel_err);
    
    let mut velocity = VEL_MULT * avg_dpress.abs();
    velocity = velocity.sqrt();
  }
}

fn random_walk_pressures() -> MeasureVal {

  let air_press_dist = Normal::new(DEFAULT_AIR_PRESS.into(), (DEFAULT_AIR_PRESS * 0.1).into() );
  let mut all_dp_pairs = Vec::new();
  let mut all_dps = Vec::new();
  
  for _sensor_instantiation_count in 0..100 {
    let actual_press = air_press_dist.sample(&mut rand::thread_rng()) as MeasureVal;
    let mut sensor_a = init_sensulator(ABS_ERR, REL_ERR, actual_press);
    let mut sensor_b = init_sensulator(ABS_ERR, REL_ERR, actual_press);
  
    let mut cluster_dps = Vec::new();
  
    for _cluster_poll_count in 0..100 {
      let dpress =  sensor_b.read() - sensor_a.read();
      cluster_dps.push(dpress);
    }
    
    let cluster_avg_dpress = statistical::mean(&cluster_dps);
    let pair = DPressPair  {
        actual_pressure: actual_press,
        dpress: cluster_avg_dpress,
    };
    all_dp_pairs.push(pair);
    all_dps.push(cluster_avg_dpress);
  }

  // for pair in all_dp_pairs {
  //   println!("{}\t{}\t{}",pair.actual_pressure, pair.dpress, (pair.dpress/pair.actual_pressure));
  // }
  
  let avg_dpress = statistical::mean(&all_dps);
  let std_dev = statistical::standard_deviation(&all_dps, Some(avg_dpress));
  let rel_err = (3 as MeasureVal) * std_dev;
  
  // println!("avg_dpress: {}\tstd_dev: {}\tpop_std_dev: {}", avg_dpress, std_dev, pop_std_dev);
  
  //velocity = sqrt ( (2/ 1.225) * dP )
  let mut velocity = VEL_MULT * avg_dpress.abs();
  velocity = velocity.sqrt();

  println!("velocity: {}", velocity);
  
  velocity
}

/*
Velocity squared = 2 dP / (air density)
velocity = sqrt ( (2/ 1.225) * dP )
air density avg = 1.225 kg/m3
*/

fn main() {
  // ever_increasing_velocity();
  sdp33_test();
  
  // let mut all_velocities = Vec::new();
  //
  // for _ in 0..1000 {
  //   let vel = random_walk_pressures();
  //   all_velocities.push(vel);
  // }
  //
  // let avg_vel = statistical::mean(&all_velocities);
  // println!("avg_vel: {} ", avg_vel );
  //
  // let std_dev = statistical::standard_deviation(&all_velocities, Some(avg_vel));
  // let rel_err = (3 as MeasureVal) * std_dev;
  // println!(" std_dev: {} rel_err: {} pct: {}", std_dev, rel_err, (rel_err / 45f32));
  
}
