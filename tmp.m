A = [1 2 3;
    4 5 6;
    7 8 9]; % Пример матрицы 3x3
[m, n] = size(A);

% Создаем пустой массив для хранения элементов в порядке обхода
result = [];

% Обходим матрицу по возрастающим диагоналям
for s = 2:(m + n)
    i_min = max(1, s - n);
    i_max = min(m, s - 1);
    
    for i = i_min:i_max
        j = s - i;
        result = [result, A(i, j)];
    end
end

disp(result);