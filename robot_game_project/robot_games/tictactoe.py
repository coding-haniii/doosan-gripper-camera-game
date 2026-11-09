WIN_LINES = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6)]

def outcome(board):
    flat = sum(board, [])
    for a,b,c in WIN_LINES:
        if flat[a] != ' ' and flat[a] == flat[b] == flat[c]:
            return flat[a]
    return 'DRAW' if ' ' not in flat else None

def best_move(board, robot='O', human='X'):
    """Perfect-play minimax for a legal board; result (row, col) or None."""
    flat = [v for row in board for v in row]
    if any(v not in ('X','O',' ') for v in flat):
        raise ValueError('Invalid cell values')
    if outcome(board) is not None:
        return None
    def mm(cells, is_robot, depth):
        state = outcome([cells[i:i+3] for i in (0,3,6)])
        if state == robot: return 10-depth, None
        if state == human: return depth-10, None
        if state == 'DRAW': return 0, None
        options = []
        for i in range(9):
            if cells[i] == ' ':
                new = cells.copy(); new[i] = robot if is_robot else human
                value,_ = mm(new, not is_robot, depth+1)
                options.append((value, i))
        score, idx = (max(options, key=lambda z: (z[0], -abs(z[1]-4))) if is_robot
                      else min(options, key=lambda z: z[0]))
        return score, idx
    _, ix = mm(flat, True, 0)
    return divmod(ix, 3)

def render_board(board, size=600):
    import cv2
    import numpy as np
    img = np.full((size, size, 3), 255, np.uint8)
    for v in (size//3, 2*size//3):
        cv2.line(img, (v, 0), (v, size-1), (0,0,0), 5)
        cv2.line(img, (0, v), (size-1, v), (0,0,0), 5)
    for r in range(3):
        for c in range(3):
            cx, cy = (2*c+1)*size//6, (2*r+1)*size//6
            if board[r][c] == 'O': cv2.circle(img, (cx,cy), 55, (0,0,0), 9)
            if board[r][c] == 'X':
                cv2.line(img,(cx-50,cy-50),(cx+50,cy+50),(0,0,0),9)
                cv2.line(img,(cx+50,cy-50),(cx-50,cy+50),(0,0,0),9)
    return img
