/////////////////////////////////
//////////Solver/////////////////
/////////////////////////////////

declare function matrix_solver{
    declare local parameter matrix.
    declare local parameter results.

    local matrix_invert_determinant is 1/(matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0]).
    
    local matrix_inverse is list(list(matrix[1][1],-matrix[0][1]),list(-matrix[1][0],matrix[0][0])).

    from{local i is 0.} until i = 2 step{set i to i+1.} do{
        from{local j is 0.} until j = 2 step{set j to j+1.} do{
	    set matrix_inverse[i][j] to matrix_invert_determinant*matrix_inverse[i][j].
	}
    }

    local desired_values is list(matrix_inverse[0][0]*results[0]+matrix_inverse[0][1]*results[1], matrix_inverse[1][0]*results[0]+matrix_inverse[1][1]*results[1]).

    return desired_values.
}