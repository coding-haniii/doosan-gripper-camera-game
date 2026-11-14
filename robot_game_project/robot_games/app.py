import argparse
import subprocess
import sys
from pathlib import Path
from runpy import run_path  # [TIC 연결] 전용 게임 파일을 경로로 불러온다.
import cv2
import yaml
from . import vision as vision_mod
from .vision import recognize_card  # [TIC 연결] 카드 인식은 app.py에서 처리한다.
from .maze import generate_maze, solve_maze, overlay
from .motion import MockRobot, GuardedDoosan, image_path_to_base

ROOT=Path(__file__).resolve().parents[1]
def cfg_load(path):
    with open(path,encoding='utf8') as f: return yaml.safe_load(f)

# [TIC 연결] 틱택토가 선택되면 전용 폴더의 run()에 실행을 넘긴다.
def tictactoe(args,robot,cfg):
    game_file = Path(__file__).with_name('tictactoe') / 'tictactoe.py'
    # 기존 robot_games.tictactoe 모듈과 이름이 겹치므로 전용 파일을 경로로 지정한다.
    game = run_path(str(game_file), run_name=f'{__package__}._tictactoe_game')
    return game['run'](args, robot, cfg)

def maze(args,robot,cfg):
    if args.image:
        img=cv2.imread(args.image,cv2.IMREAD_GRAYSCALE)
        if img is None:raise ValueError(f'Could not load {args.image}')
        if args.start is None or args.goal is None:
            raise ValueError('--start X Y and --goal X Y required for arbitrary maze')
        start,goal=tuple(args.start),tuple(args.goal)
    else:
        img,start,goal=generate_maze(args.seed,args.cells)
    path=solve_maze(img,start,goal,clearance=cfg['vision']['maze_clearance_px'],
                    threshold=cfg['vision']['maze_threshold'],stride=cfg['vision']['maze_step_px'])
    out=ROOT/'out';out.mkdir(exist_ok=True)
    cv2.imwrite(str(out/'maze_input.png'),img)
    cv2.imwrite(str(out/'maze_solved.png'),overlay(img,path))
    print(f'Maze solved: {len(path)} waypoints; start={start}; goal={goal}')
    if isinstance(robot,MockRobot):robot.draw(path,'maze')
    else:robot.draw(image_path_to_base(path,img.shape,cfg['robot']),'maze')
    robot.home()

# [실기] 게임 앞뒤로 펜 거치대에서 펜 부품을 집고 반납한다(robot_games.pen_holder).
# DSR_ROBOT2 는 프로세스당 연결 하나라서(미로는 maze_executor 가 따로 연결) 별도 프로세스로 실행한다.
def pen_step(action, args, cfg, run=subprocess.run):
    """True=펜을 쥔 상태로 게임 진행 가능. robot.pen_from_holder 가 false 면 아무것도 하지 않음(펜 미리 물림)."""
    if not cfg['robot'].get('pen_from_holder'):
        return True
    if args.mode != 'real':
        print(f'[MOCK] pen {action} (거치대)')
        return True
    print(f'=== 펜 {"집기" if action == "pick" else "반납"} (거치대) ===', flush=True)
    rc = run([sys.executable, '-m', 'robot_games.pen_holder', action, '--execute-motion', '--yes'], cwd=str(ROOT)).returncode
    if rc == 0 or (action == 'pick' and rc == 3) or (action == 'return' and rc == 4):   # 3: 이미 쥐고 있음, 4: 빈 손
        return True
    print(f'[경고] 펜 {action} 실패 (종료코드 {rc})' + (' — 로봇이 펜을 쥔 채일 수 있음. 확인 후 '
          'python -m robot_games.pen_holder return --execute-motion' if action == 'return' else ''), file=sys.stderr)
    return False

def main():
    parser=argparse.ArgumentParser(description='Doosan E0509 camera games (mock-first)')
    parser.add_argument('--config',default=str(ROOT/'configs/default.yaml'))
    parser.add_argument('--mode',choices=['mock','real'],default='mock')
    parser.add_argument('--game',choices=['1','2','3','tictactoe','maze','exit','camera'],default='1')
    parser.add_argument('--image',help='Photo of game board or maze')
    parser.add_argument('--card-image',help='Photo of action card for recognition')
    parser.add_argument('--truth',choices=['tictactoe','maze','exit'],help='지금 보여주는 카드의 정답(학습용). out/ocr_log.jsonl 에 함께 기록됨')
    parser.add_argument('--start',type=int,nargs=2,metavar=('X','Y'))
    parser.add_argument('--goal',type=int,nargs=2,metavar=('X','Y'))
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--cells',type=int,default=8)
    parser.add_argument('--execute-motion',action='store_true',help='Additional physical motion gate')
    parser.add_argument('--paper-maze',action='store_true',help='maze/card 2: run chained camera->JSON->CSV->robot pipeline (robot_games.maze_game)')
    parser.add_argument('--quiet-ocr',action='store_true',help='OCR 디버그 로그/out/debug 이미지 저장 끄기')
    parser.add_argument('--paper-maze-source',choices=['camera','image','demo'],default='camera')
    args=parser.parse_args();cfg=cfg_load(args.config)
    if args.truth: vision_mod.TRUTH=args.truth
    if args.quiet_ocr: vision_mod.DEBUG=False
    game={'1':'tictactoe','2':'maze','3':'exit'}.get(args.game,args.game)
    if args.card_image:
        img=cv2.imread(args.card_image)
        if img is None:raise ValueError('Card image unreadable')
        game=recognize_card(img)
        if not game:raise ValueError('Card unrecognized: no action performed (out/debug/ 의 이미지와 [card] 로그를 확인하세요)')
        print('Card detected:',game)
    if game=='camera':
        from .camera_select import ReconnectingCamera   # [실기] RealSense 컬러 자동 + 끊기면 다시 연결
        cap=ReconnectingCamera(int(cfg['camera'].get('index',0)))
        if not cap.isOpened():raise RuntimeError('Cannot open camera')
        cam=cfg.get('camera',{})
        if cam.get('width'):cap.set(cv2.CAP_PROP_FRAME_WIDTH,cam['width'])
        if cam.get('height'):cap.set(cv2.CAP_PROP_FRAME_HEIGHT,cam['height'])
        print('Show action card inside the green guide; press SPACE to capture, Q to quit')
        try:
            for _ in range(15):cap.read()     # 자동노출/초점 안정화를 위해 초기 프레임 버림
            while True:
                ok,frame=cap.read()
                if not ok:raise RuntimeError('Camera frame failed')
                # 가이드 박스: 화면 중앙 70% (카드를 화면의 30~60% 이상 채우고 평평하게 두기)
                view=frame.copy();fh,fw=view.shape[:2]
                cv2.rectangle(view,(int(fw*.15),int(fh*.15)),(int(fw*.85),int(fh*.85)),(0,200,0),2)
                cv2.imshow('Action card (SPACE capture/Q exit)',view)
                key=cv2.waitKey(20)&0xff
                if key==ord('q'):game='exit';break
                if key==32:
                    out=ROOT/'out';out.mkdir(exist_ok=True)
                    cv2.imwrite(str(out/'capture.png'),frame)   # 캡처 원본 저장 (재현/디버깅용)
                    game=recognize_card(frame)                  # 전체 프레임 전달: vision에서 카드 검출/크롭
                    if game:
                        print('Detected:',game)
                        # 카메라 모드에서 maze 카드가 인식되면 paper_maze 파이프라인 자동 활성화
                        if game == 'maze':
                            args.paper_maze = True
                        break
                    print('Card not recognized. Try again. (out/capture.png, out/debug/ 확인)')
        finally:
            cap.release();cv2.destroyAllWindows()
    if args.mode=='real' and not args.execute_motion:
        raise PermissionError('Physical mode requires --execute-motion and separate calibrated+armed YAML')
    # [실기] 미로·틱택토는 시작 전에 펜을 거치대에서 집고, 끝나면(오류여도) 반납한다
    holding=False
    if game=='exit':pen_step('return',args,cfg)   # [실기] 앞선 반납이 실패해 펜을 쥐고 있으면 여기서 반납(빈 손이면 그냥 넘어감)
    if game in ('maze','tictactoe'):
        if not pen_step('pick',args,cfg):raise SystemExit('펜을 집지 못해 게임을 시작하지 않음')
        holding=True
    try:
        if game=='maze' and args.paper_maze:      # 카메라 종이 미로: 비전→JSON→CSV→로봇 연속 실행
            from .maze_game import run as run_paper_maze
            argv=['--mode',args.mode,'--source',args.paper_maze_source,'--seed',str(args.seed)]
            if args.execute_motion: argv+=['--execute-motion','--yes']   # [실기] 펜 집기처럼 --execute-motion 을 승인으로 보고 RUN 입력 생략
            if args.image: argv+=['--image',args.image]
            rc=run_paper_maze(argv)
            if rc:raise SystemExit(rc)
            return
        robot=MockRobot() if args.mode=='mock' else GuardedDoosan(cfg['robot'])
        try:
            if game=='tictactoe':tictactoe(args,robot,cfg)
            elif game=='maze':maze(args,robot,cfg)
            elif game=='exit':print('Exit card: safe exit');robot.home()
            else:raise ValueError('Unknown game, no motion performed')
        finally:
            if hasattr(robot,'close'):robot.close()   # 반납(별도 프로세스) 전에 이 연결을 닫는다
    finally:
        if holding:pen_step('return',args,cfg)

if __name__=='__main__':main()
