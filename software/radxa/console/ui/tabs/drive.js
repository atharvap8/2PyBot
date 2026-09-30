/* tabs/drive.js — Drive configuration.
 * Owns firmware parameter group "DRIVE". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'drive', title: 'Drive', order: 30,
    render(el, ctx) {
      ctx.renderGroup(el, 'DRIVE', {
        title: 'Drive',
        hint: 'Speed scaling and stick shaping. SNAPIN is the cone around straight-ahead where steering is forced to zero — raise it if full-forward still drifts.'.replace(/^'|'$/g, '')
      });
    }
  });
}
