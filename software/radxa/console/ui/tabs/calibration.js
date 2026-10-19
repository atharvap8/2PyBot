import { mountCalibration } from '../calibration_ui.js';
export default function register(registerTab){
  let dispose=null;
  registerTab({id:'calibration',title:'Calibration',order:2.5,
    render(el){dispose=mountCalibration(el);},onLeave(){dispose?.();dispose=null;}});
}
