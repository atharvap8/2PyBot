/* tabs/imu.js — IMU configuration.
 * Owns firmware parameter group "IMU". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'imu', title: 'IMU', order: 60,
    render(el, ctx) {
      ctx.renderGroup(el, 'IMU', {
        title: 'IMU',
        hint: 'Filter cutoff, Mahony gains and the three sign conventions. Verify a sign by tilting the robot by hand and watching the telemetry before trusting it.'.replace(/^'|'$/g, '')
      });
    }
  });
}
