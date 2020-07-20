_mogs2()
{
    local argc cur opts
    COMPREPLY=()

    argc=${COMP_CWORD}
    cur="${COMP_WORDS[argc]}"
   
    args=("${COMP_WORDS[@]}")
    unset args[${#args[@]}-1]

    opts=("$(MogsCompletion  ${args[*]:1} args)")
    reply=($opts)

    case ${reply[0]} in
	file:)
		_filedir "@(${reply[1]})"
		;;
	folder:)
		_filedir "@()"
		;;
	*)
		COMPREPLY=( $(compgen -W "$opts" -- $cur ) )
		;;
    esac	

}
complete -F _mogs2 mogs2
