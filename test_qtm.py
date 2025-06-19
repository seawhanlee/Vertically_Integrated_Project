# test_qtm.py
import asyncio
import qtm_rt

async def test_qtm_connection():
    try:
        # QTM 인스턴스 검색
        async for qtm_instance in qtm_rt.Discover('0.0.0.0'):
            print(f"QTM 발견: {qtm_instance.host}:{qtm_instance.port}")
            
            # 연결 시도
            connection = await qtm_rt.connect(qtm_instance.host)
            print("QTM 연결 성공!")
            
            # 6DOF 바디 정보 확인
            params = await connection.get_parameters(parameters=['6d'])
            print("6DOF 설정 가져오기 성공")
            
            await connection.disconnect()
            return
            
    except Exception as e:
        print(f"QTM 연결 실패: {e}")

# 테스트 실행
asyncio.run(test_qtm_connection())