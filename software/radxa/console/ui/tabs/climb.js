/* tabs/climb.js — Climb configuration.
 * Owns firmware parameter group "CLIMB". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'climb', title: 'Climb', order: 70,
    render(el, ctx) {
      ctx.renderGroup(el, 'CLIMB', {
        title: 'Climb',
        hint: 'Ramp/incline mode. Engaged with D-pad DOWN on the pad.'.replace(/^'|'$/g, '')
      });
    }
  });
}
