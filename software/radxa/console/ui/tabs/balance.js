/* tabs/balance.js — Balance configuration.
 * Owns firmware parameter group "BALANCE". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'balance', title: 'Balance', order: 10,
    render(el, ctx) {
      ctx.renderGroup(el, 'BALANCE', {
        title: 'Balance',
        hint: 'The LQR/LQI gain vector and the stiff-hold set. K4 is gyro-rate damping — its SIGN is the single most dangerous value on the robot. Change one gain at a time.'.replace(/^'|'$/g, '')
      });
    }
  });
}
