/* tabs/motor.js — BLDC motor configuration.
 * Owns firmware parameter group "MOTOR". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'motor', title: 'BLDC Motors', order: 20,
    render(el, ctx) {
      ctx.renderGroup(el, 'MOTOR', {
        title: 'BLDC Motors',
        hint: 'Geometry and driver limits. WHEELR is the LOADED rolling radius: measure it (push 2 m, read encoder delta) rather than halving the moulded diameter. Everything derived updates live below.'.replace(/^'|'$/g, '')
      });
    }
  });
}
