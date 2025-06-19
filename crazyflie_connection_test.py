# test_connection.py
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# 드라이버 초기화
cflib.crtp.init_drivers()

# URI 설정 (환경에 맞게 수정)
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

print(f"연결 시도: {uri}")

try:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Crazyflie 연결 성공!")
        
        # 기본 정보 확인
        cf = scf.cf
        print(f"배터리 전압: {cf.param.get_value('pm.vbat')}")
        
except Exception as e:
    print(f"연결 실패: {e}")