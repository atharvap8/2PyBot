"""Band migration uses a simulated NetworkManager/radio; never touches host Wi-Fi."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

SOURCE=Path(__file__).resolve().parents[1]/'scripts/radxa/deploy.py'
SPEC=importlib.util.spec_from_file_location('hotspot_deploy_test',SOURCE)
deploy=importlib.util.module_from_spec(SPEC);sys.modules[SPEC.name]=deploy;SPEC.loader.exec_module(deploy)
UUID='aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee'
OTHER_UUID='11111111-2222-3333-4444-555555555555'


class HotspotTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name);repo=root/'repo';settings=repo/'scripts/radxa/hotspot.json'
        settings.parent.mkdir(parents=True)
        settings.write_text(json.dumps({'connection':'2pybot-ap','band':'bg','channel':6}))
        self.settings=settings;self.other_wifi=False
        self.manager=deploy.Deployment(repo,'radxa',root/'state','http://localhost',root/'system')
        self.manager.commit='a'*40;self.commands=[];self.radio=(36,5180)
        self.fields={'connection.uuid':UUID,'connection.id':'2pybot-ap','connection.type':'802-11-wireless','802-11-wireless.mode':'ap',
            'GENERAL.STATE':'activated','GENERAL.DEVICES':'wlan0','802-11-wireless.band':'a','802-11-wireless.channel':'36'}
        self.manager.hotspot_command=self.command
    def command(self,*args):
        self.commands.append(args)
        if args[0]=='iw':return 'Interface wlan0\n channel %d (%d MHz), width: 20 MHz'%self.radio
        if args[1]=='--get-values':
            if args[2]=='UUID':return UUID+'\n'+OTHER_UUID
            if args[2]=='connection.type' and args[-1]==OTHER_UUID and not self.other_wifi:return '802-3-ethernet'
            return self.fields[args[2]]
        if args[1:3]==('connection','modify'):
            self.assertEqual(args[3:5],('uuid',UUID))
            self.assertEqual(args[5],'802-11-wireless.band');self.assertEqual(args[7],'802-11-wireless.channel')
            self.fields[args[5]]=args[6];self.fields[args[7]]=args[8]
        elif args[1:5]==('--wait','30','connection','up'):
            channel=int(self.fields['802-11-wireless.channel'])
            self.radio=(channel,2407+5*channel if self.fields['802-11-wireless.band']=='bg' else 5000+5*channel)
        else:raise AssertionError(args)
        return ''
    def writes(self):return [c for c in self.commands if c[:3]==('nmcli','connection','modify')]
    def test_migrates_only_band_channel_and_verifies_live_2437_mhz(self):
        settings=self.manager.prepare_hotspot();result=self.manager.apply_hotspot(settings)
        self.assertEqual((result['band'],result['channel']),('bg',6));self.assertEqual(self.radio,(6,2437))
        self.assertEqual(len(self.writes()),1)
        self.assertFalse(any('delete' in c or 'wifi-sec.psk' in c or 'ipv4.method' in c for c in self.commands))
        backup=self.manager.state_dir/'hotspot-backups'/('a'*40+'.json')
        self.assertEqual(json.loads(backup.read_text())['previous_band'],'a')
    def test_already_on_requested_band_is_not_restarted(self):
        self.fields.update({'802-11-wireless.band':'bg','802-11-wireless.channel':'6'});self.radio=(6,2437)
        self.manager.apply_hotspot(self.manager.prepare_hotspot())
        self.assertFalse(self.writes());self.assertFalse(any('up' in c for c in self.commands))
    def test_saved_profile_match_does_not_hide_a_still_running_5ghz_radio(self):
        self.fields.update({'802-11-wireless.band':'bg','802-11-wireless.channel':'6'})
        self.manager.apply_hotspot(self.manager.prepare_hotspot())
        self.assertEqual(self.radio,(6,2437));self.assertEqual(len(self.writes()),1)
    def test_station_connection_is_rejected_before_any_change(self):
        self.fields['802-11-wireless.mode']='infrastructure'
        with self.assertRaisesRegex(RuntimeError,'not a Wi-Fi access point'):self.manager.prepare_hotspot()
        self.assertFalse(self.writes())
    def test_rollback_recovers_original_5ghz_profile_and_radio(self):
        settings=self.manager.prepare_hotspot();self.manager.apply_hotspot(settings)
        self.manager.restore_hotspot(settings)
        self.assertEqual(self.radio,(36,5180));self.assertEqual(self.fields['802-11-wireless.band'],'a')
    def test_auto_selection_uses_the_unique_ap_and_leaves_ethernet_alone(self):
        self.settings.write_text(json.dumps({'connection':None,'band':'bg','channel':6}))
        selected=self.manager.prepare_hotspot();self.manager.apply_hotspot(selected)
        self.assertEqual(selected['uuid'],UUID)
        self.assertTrue(all(c[4]==UUID for c in self.writes()))
    def test_ambiguous_active_hotspots_require_an_explicit_profile(self):
        self.settings.write_text(json.dumps({'connection':None,'band':'bg','channel':6}));self.other_wifi=True
        with self.assertRaisesRegex(RuntimeError,'exactly one'):self.manager.prepare_hotspot()
        self.assertFalse(self.writes())


if __name__=='__main__':unittest.main()
