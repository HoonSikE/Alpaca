package com.example.taxi.ui.mypage.favorites

import android.util.Log
import android.view.View
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.databinding.FragmentFavoritesBinding
import com.example.taxi.ui.home.user.DestinationListAdapter
import com.example.taxi.ui.home.user.FavoritesAdapter
import com.example.taxi.ui.home.user.FavoritesDialogFragment
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class FavoritesFragment : BaseFragment<FragmentFavoritesBinding>(R.layout.fragment_favorites) {
    private val userHomeViewModel : UserHomeViewModel by viewModels()

    private lateinit var updateFavoritesAdapter: UpdateFavoritesAdapter
    private var favorites : MutableList<Favorites> = mutableListOf()
    private var addressFavorites = ""

    // 즐겨찾기 item 삭제
    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        addressFavorites = address
        showFavoritesDialog(address)
    }

    override fun init() {
        initAdapter()
        setOnClickListeners()
        observerData()
    }

    private fun initAdapter() {
        userHomeViewModel.getFavorites()
        updateFavoritesAdapter = UpdateFavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
        }
        binding.recyclerviewFavorites.apply {
            adapter = updateFavoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun setOnClickListeners(){
        binding.imgFavoritesBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.buttonFavoritesUpdate.setOnClickListener{
            if(binding.searchFavoritesSearch.query.toString() != ""){
                val addfavorite = Favorites("", "", binding.searchFavoritesSearch.query.toString(), "")
                favorites.add(addfavorite)
                if(favorites.size == 1){
                    userHomeViewModel.addFavorites(favorites)
                }else{
                    userHomeViewModel.updateFavorites(favorites)
                }
                userHomeViewModel.getFavorites()
            }
        }
    }

    private fun observerData() {
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textNoContentFavorites.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    favorites = state.data.toMutableList()
                    updateFavoritesAdapter.updateList(favorites)
                    binding.recyclerviewFavorites.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textNoContentFavorites.hide()
                }
            }
        }
    }

    // Dialog를 보여줌과 동시에 삭제 명령을 내린다.
    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment { favoritesListener(address) }.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private val favoritesListener: (address: String) -> Unit = {
        if(favorites.size < 2) {
            userHomeViewModel.deleteFavorites()
        }else{
            for(i in 0 until favorites.size){
                if(favorites[i].address == addressFavorites) {
                    favorites.removeAt(i)
                    userHomeViewModel.updateFavorites(favorites)
                }
            }
        }
        userHomeViewModel.getFavorites()
    }
}